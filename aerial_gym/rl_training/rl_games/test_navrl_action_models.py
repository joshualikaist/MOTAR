"""CPU tests for bounded NavRL PPO action distributions."""

import os
import unittest

import torch
import torch.nn as nn
from torch.distributions import Normal, TanhTransform, TransformedDistribution

from rl_games.algos_torch import models

from navrl_action_models import (
    NavRLFixedGaussianModel,
    NavRLSquashedGaussianModel,
    NavRLTruncatedGaussianModel,
)


class _DummyA2CNetwork(nn.Module):
    def __init__(self, action_dim=4):
        super().__init__()
        self.mu_bias = nn.Parameter(torch.zeros(action_dim))
        # Preserve the same parameter/state_dict surface as the real custom Transformer.
        self.sigma = nn.Parameter(torch.zeros(action_dim))

    def forward(self, input_dict):
        batch = input_dict["obs"].shape[0]
        mu = self.mu_bias.expand(batch, -1)
        logstd = self.sigma.expand(batch, -1)
        value = mu[:, :1] * 0.0
        return mu, logstd, value, None

    def get_aux_loss(self):
        return None

    def is_rnn(self):
        return False

    def get_value_layer(self):
        return None

    def get_default_rnn_state(self):
        return None


def _make_network(model_cls):
    return model_cls.Network(
        _DummyA2CNetwork(),
        obs_shape=(3,),
        normalize_value=False,
        normalize_input=False,
        value_size=1,
    )


class ActionModelTests(unittest.TestCase):
    def setUp(self):
        self._old_std = os.environ.get("NAVRL_ACTION_STD")
        self._old_mu_scale = os.environ.get("NAVRL_ACTION_MU_SCALE")
        os.environ["NAVRL_ACTION_STD"] = "0.35,0.35,0.05,0.08"
        os.environ["NAVRL_ACTION_MU_SCALE"] = "1,1,1,1"

    def tearDown(self):
        if self._old_std is None:
            os.environ.pop("NAVRL_ACTION_STD", None)
        else:
            os.environ["NAVRL_ACTION_STD"] = self._old_std
        if self._old_mu_scale is None:
            os.environ.pop("NAVRL_ACTION_MU_SCALE", None)
        else:
            os.environ["NAVRL_ACTION_MU_SCALE"] = self._old_mu_scale

    def test_bounded_models_are_finite_and_strictly_bounded(self):
        obs = torch.zeros(100_000, 3)

        for model_cls in (NavRLSquashedGaussianModel, NavRLTruncatedGaussianModel):
            network = _make_network(model_cls)
            rollout = network({"obs": obs, "is_train": False})
            actions = rollout["actions"]
            self.assertTrue(bool(torch.isfinite(actions).all()))
            self.assertTrue(bool((actions > -1.0).all()))
            self.assertTrue(bool((actions < 1.0).all()))
            # At zero mean and lateral std=.35, neither bounded model should manufacture a
            # boundary atom. The generous limit is deterministic across Torch RNG versions.
            self.assertLess(
                float((actions[:, 1].abs() >= 0.98).float().mean()), 1.0e-3
            )

            update = network(
                {"obs": obs[:512], "is_train": True, "prev_actions": actions[:512]}
            )
            loss = update["prev_neglogp"].mean() - 1.0e-4 * update["entropy"].mean()
            loss.backward()
            self.assertTrue(bool(torch.isfinite(network.a2c_network.mu_bias.grad).all()))

    def test_fixed_gaussian_tail_matches_small_std(self):
        torch.manual_seed(7)
        network = _make_network(NavRLFixedGaussianModel)
        rollout = network({"obs": torch.zeros(300_000, 3), "is_train": False})
        lateral_oob = float((rollout["actions"][:, 1].abs() > 1.0).float().mean())
        # Theoretical two-sided tail for N(0, .35): about 0.00427.
        self.assertGreater(lateral_oob, 0.0035)
        self.assertLess(lateral_oob, 0.0051)

    def test_custom_wrappers_preserve_legacy_state_dict_keys(self):
        legacy = models.ModelA2CContinuousLogStd.Network(
            _DummyA2CNetwork(),
            obs_shape=(3,),
            normalize_value=False,
            normalize_input=False,
            value_size=1,
        )
        expected = set(legacy.state_dict())
        for model_cls in (
            NavRLFixedGaussianModel,
            NavRLSquashedGaussianModel,
            NavRLTruncatedGaussianModel,
        ):
            self.assertEqual(set(_make_network(model_cls).state_dict()), expected)

    def test_squashed_deterministic_action_is_post_tanh(self):
        network = _make_network(NavRLSquashedGaussianModel)
        with torch.no_grad():
            network.a2c_network.mu_bias.copy_(torch.tensor([0.0, 0.5, -1.0, 2.0]))
        result = network({"obs": torch.zeros(2, 3), "is_train": False})
        expected = torch.tanh(network.a2c_network.mu_bias)
        self.assertTrue(
            torch.allclose(result["deterministic_actions"][0], expected, atol=2.0e-6)
        )

    def test_squashed_logprob_matches_torch_transform(self):
        network = _make_network(NavRLSquashedGaussianModel)
        latent_mu = torch.tensor([[0.1, -0.4, 0.2, 1.0]])
        sigma = network._std_like(latent_mu)
        base = Normal(latent_mu, sigma)
        action = torch.tanh(torch.tensor([[0.2, -0.7, 0.19, 0.8]]))
        actual = network._neglogp(base, action)
        transformed = TransformedDistribution(base, [TanhTransform(cache_size=1)])
        expected = -transformed.log_prob(action).sum(dim=-1)
        self.assertTrue(torch.allclose(actual, expected, atol=2.0e-5, rtol=2.0e-5))

    def test_lateral_mu_warmstart_scaling(self):
        os.environ["NAVRL_ACTION_MU_SCALE"] = "1,0.4,1,1"
        network = _make_network(NavRLSquashedGaussianModel)
        with torch.no_grad():
            network.a2c_network.mu_bias.copy_(torch.tensor([2.0, 2.0, 2.0, 2.0]))
        result = network({"obs": torch.zeros(1, 3), "is_train": False})
        expected = torch.tanh(torch.tensor([2.0, 0.8, 2.0, 2.0]))
        self.assertTrue(
            torch.allclose(result["deterministic_actions"][0], expected, atol=2.0e-6)
        )

    def test_truncated_pdf_is_normalized(self):
        network = _make_network(NavRLTruncatedGaussianModel)
        raw_mu = torch.tensor([[-2.0, -0.3, 0.4, 2.0]])
        location, scale, _alpha, _beta, _cdf_low, normalizer = (
            network._distribution_parameters(raw_mu)
        )
        grid = torch.linspace(-1.0 + 1.0e-6, 1.0 - 1.0e-6, 20_001)
        action = grid[:, None].expand(-1, 4)
        log_pdf = Normal(location, scale).log_prob(action) - torch.log(normalizer)
        integral = torch.trapz(torch.exp(log_pdf), grid, dim=0)
        self.assertTrue(torch.allclose(integral, torch.ones_like(integral), atol=2.0e-3))


if __name__ == "__main__":
    unittest.main()
