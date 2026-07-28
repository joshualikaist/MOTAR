"""rl_games player compatibility for bounded NavRL action models."""

import torch

from rl_games.algos_torch.players import PpoPlayerContinuous, rescale_actions
from rl_games.common.tr_helpers import unsqueeze_obs


class NavRLPpoPlayerContinuous(PpoPlayerContinuous):
    """Use a model-provided bounded deterministic action when one is available.

    rl_games 1.6.5's continuous player always selects ``res_dict['mus']`` for deterministic
    evaluation.  For a tanh-squashed policy that tensor is the pre-tanh Gaussian mean (also needed
    for PPO's latent KL), so returning it directly evaluates a different controller.  Legacy models
    do not provide ``deterministic_actions`` and retain the stock behavior.
    """

    def get_action(self, obs, is_deterministic=False):
        if not self.has_batch_dimension:
            obs = unsqueeze_obs(obs)
        obs = self._preproc_obs(obs)
        input_dict = {
            "is_train": False,
            "prev_actions": None,
            "obs": obs,
            "rnn_states": self.states,
        }
        with torch.no_grad():
            res_dict = self.model(input_dict)

        sampled_action = res_dict["actions"]
        deterministic_action = res_dict.get("deterministic_actions", res_dict["mus"])
        self.states = res_dict["rnn_states"]
        current_action = deterministic_action if is_deterministic else sampled_action
        if not self.has_batch_dimension:
            current_action = torch.squeeze(current_action.detach())

        if self.clip_actions:
            return rescale_actions(
                self.actions_low,
                self.actions_high,
                torch.clamp(current_action, -1.0, 1.0),
            )
        return current_action
