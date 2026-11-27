"""U3: independent CV moments, Joseph/expectation algebra and explicit RNG tests."""
import ast
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import belief  # noqa: E402

assert str(Path(belief.__file__).resolve()).startswith(str(ROOT) + "/")


def generator(seed=20261019):
    result = torch.Generator(device="cpu")
    result.manual_seed(seed)
    return result


def distribution(n=2):
    mu = torch.arange(n * 6, dtype=torch.float64).reshape(n, 6) * .03
    a = torch.randn(n, 6, 6, dtype=torch.float64, generator=generator()) * .15
    cov = a @ a.transpose(-1, -2) + .2 * torch.eye(6, dtype=torch.float64)
    return mu, cov


def scalar_predict_oracle(mu, cov, dt, process, horizon):
    """Entry-wise CV equations; no call to the implementation or matrix F product."""
    means, covariances = [], []
    m, p = mu.tolist(), cov.tolist()
    q = process.tolist()
    for _ in range(horizon):
        m = [m[i] + (dt * m[i + 3] if i < 3 else 0.) for i in range(6)]
        p = [[p[i][j] + (dt * p[i + 3][j] if i < 3 else 0.)
              + (dt * p[i][j + 3] if j < 3 else 0.)
              + (dt * dt * p[i + 3][j + 3] if i < 3 and j < 3 else 0.)
              + q[i][j] for j in range(6)] for i in range(6)]
        means.append(m)
        covariances.append(p)
    return torch.tensor(means, dtype=torch.float64), torch.tensor(covariances, dtype=torch.float64)


def update_fixture():
    _, cov = distribution(2)
    pred = cov[:, None, None].repeat(1, 3, 4, 1, 1)
    R = torch.diag(torch.tensor([.04, .09, .16], dtype=torch.float64))[None].repeat(2, 1, 1)
    return pred, R


class BeliefTest(unittest.TestCase):
    def test_tracker_q_matches_frozen_source_constants_without_import(self):
        # Source: navrl_perception.py:653-662. Importing aerial_gym triggers
        # isaacgym through aerial_gym/__init__.py:2, so mirror constants only.
        actual = belief.tracker_Q(.1, dtype=torch.float64)
        expected = torch.diag(torch.tensor([.00025] * 3 + [.025] * 3, dtype=torch.float64))
        torch.testing.assert_close(actual, expected, atol=1e-17, rtol=0)
        tree = ast.parse((ROOT / "aerial_gym/task/navrl_task/navrl_perception.py").read_text())
        tracker = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                       and n.name == "BatchedConstantVelocityTracker")
        init = next(n for n in tracker.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        coefficient_assignments = {
            n.targets[0].id: n.value for n in init.body
            if isinstance(n, ast.Assign) and len(n.targets) == 1
            and isinstance(n.targets[0], ast.Name) and n.targets[0].id in ("q_pos", "q_vel")
        }
        constants = {name: [x.value for x in ast.walk(expr) if isinstance(x, ast.Constant)]
                     for name, expr in coefficient_assignments.items()}
        self.assertEqual(constants, {"q_pos": [.025], "q_vel": [.25]})
        for dt in (.05, .2):
            torch.testing.assert_close(belief.tracker_Q(dt, dtype=torch.float64).diagonal(),
                                       torch.tensor([.025 * dt * dt] * 3 + [.25 * dt] * 3,
                                                    dtype=torch.float64), atol=0, rtol=0)

    def test_prediction_matches_independent_scalar_cv_oracle(self):
        mu, cov = distribution()
        # Independently fixed noise constants above; do not call tracker_Q here.
        q = torch.diag(torch.tensor([.00025] * 3 + [.025] * 3, dtype=torch.float64))
        means, predicted = belief.predict(mu, cov, .1, q, 10)
        self.assertEqual(tuple(means.shape), (2, 10, 6))
        self.assertEqual(tuple(predicted.shape), (2, 10, 6, 6))
        for e in range(2):
            ref_mu, ref_cov = scalar_predict_oracle(mu[e], cov[e], .1, q, 10)
            torch.testing.assert_close(means[e], ref_mu, atol=1e-14, rtol=0)
            torch.testing.assert_close(predicted[e], ref_cov, atol=1e-14, rtol=0)

    def test_prediction_does_not_assume_each_variance_increases(self):
        mu = torch.zeros(1, 6, dtype=torch.float64)
        cov = torch.eye(6, dtype=torch.float64)[None]
        cov[0, 0, 3] = cov[0, 3, 0] = -.9
        q = torch.zeros(6, 6, dtype=torch.float64)
        _, pred = belief.predict(mu, cov, .1, q, 1)
        self.assertLess(float(pred[0, 0, 0, 0]), float(cov[0, 0, 0]))
        self.assertGreater(float(torch.linalg.eigvalsh(pred).min()), 0.)

    def test_batched_q_and_source_tensors_are_not_modified(self):
        mu, cov = distribution()
        q = torch.stack((torch.eye(6, dtype=torch.float64) * .01,
                         torch.eye(6, dtype=torch.float64) * .02))
        old = [x.clone() for x in (mu, cov, q)]
        _, prediction = belief.predict(mu, cov, .1, q, 2)
        for before, after in zip(old, (mu, cov, q)):
            self.assertTrue(torch.equal(before, after))
        for e in range(2):
            _, ref = scalar_predict_oracle(mu[e], cov[e], .1, q[e], 2)
            torch.testing.assert_close(prediction[e], ref, atol=1e-14, rtol=0)

    def test_zero_probability_exactly_preserves_given_prediction(self):
        pred, R = update_fixture()
        p = torch.zeros(2, 3, 4, dtype=torch.float64)
        result = belief.expected_cov(pred, p, R)
        self.assertTrue(torch.equal(result, pred))
        self.assertNotEqual(result.data_ptr(), pred.data_ptr())
        # No camera update needs no invertible innovation matrix.
        zero = torch.zeros_like(pred)
        self.assertTrue(torch.equal(belief.expected_cov(zero, p, torch.zeros_like(R)), zero))

    def test_probability_one_matches_independent_joseph_update(self):
        pred, R = update_fixture()
        p = torch.ones(2, 3, 4, dtype=torch.float64)
        result = belief.expected_cov(pred, p, R)
        H = torch.cat((torch.eye(3, dtype=torch.float64), torch.zeros(3, 3, dtype=torch.float64)), 1)
        for e in range(2):
            for k in range(3):
                for j in range(4):
                    prior = pred[e, k, j]
                    gain = prior @ H.t() @ torch.inverse(H @ prior @ H.t() + R[e])
                    ikh = torch.eye(6, dtype=torch.float64) - gain @ H
                    joseph = ikh @ prior @ ikh.t() + gain @ R[e] @ gain.t()
                    torch.testing.assert_close(result[e, k, j], joseph, atol=1e-9, rtol=0)

    def test_fractional_update_matches_bpod_adaptation_and_psd(self):
        pred, R = update_fixture()
        p = torch.linspace(0, 1, 24, dtype=torch.float64).reshape(2, 3, 4)
        old = [x.clone() for x in (pred, p, R)]
        result = belief.expected_cov(pred, p, R)
        for e in range(2):
            prior = pred[e]
            gain = prior[..., :, :3] @ torch.inverse(prior[..., :3, :3] + R[e])
            direct = prior - p[e, :, :, None, None] * gain @ prior[..., :3, :]
            torch.testing.assert_close(result[e], direct, atol=1e-12, rtol=0)
        self.assertGreaterEqual(float(torch.linalg.eigvalsh(result).min()), -1e-12)
        self.assertGreaterEqual(float(torch.linalg.eigvalsh(pred - result).min()), -1e-12)
        torch.testing.assert_close(result, result.transpose(-1, -2), atol=1e-14, rtol=0)
        for before, after in zip(old, (pred, p, R)):
            self.assertTrue(torch.equal(before, after))

    def test_mixed_zero_probability_does_not_invert_its_singular_scene(self):
        pred, R = update_fixture()
        pred[0].zero_()
        R[0].zero_()
        p = torch.ones(2, 3, 4, dtype=torch.float64)
        p[0].zero_()
        result = belief.expected_cov(pred, p, R)
        self.assertTrue(torch.equal(result[0], pred[0]))
        self.assertTrue(bool(torch.isfinite(result).all()))

    def test_sampling_matches_future_cv_mean_and_covariance(self):
        mu, cov = distribution(1)
        q = torch.diag(torch.tensor([.00025] * 3 + [.025] * 3, dtype=torch.float64))
        paths = belief.sample(mu, cov, q, 32768, 3, generator(), dt=.1)
        self.assertEqual(tuple(paths.shape), (1, 32768, 3, 6))
        mean_ref, cov_ref = scalar_predict_oracle(mu[0], cov[0], .1, q, 3)
        for j in range(3):
            values = paths[0, :, j]
            mean = values.mean(0)
            centered = values - mean
            empirical = centered.t() @ centered / (values.shape[0] - 1)
            mean_se = (cov_ref[j].diagonal() / values.shape[0]).sqrt()
            self.assertTrue(bool(((mean - mean_ref[j]).abs() <= 6 * mean_se).all()))
            diagonal = cov_ref[j].diagonal()
            cov_se = ((cov_ref[j].square() + diagonal[:, None] * diagonal[None])
                      / (values.shape[0] - 1)).sqrt()
            self.assertTrue(bool(((empirical - cov_ref[j]).abs() <= 6 * cov_se).all()))

    def test_sampling_uses_explicit_generator_and_preserves_global_state(self):
        mu, cov = distribution()
        q = torch.eye(6, dtype=torch.float64) * .01
        old = [x.clone() for x in (mu, cov, q)]
        global_rng = torch.get_rng_state().clone()
        g = generator()
        before_local = g.get_state().clone()
        a = belief.sample(mu, cov, q, 16, 3, g)
        b = belief.sample(mu, cov, q, 16, 3, generator())
        self.assertTrue(torch.equal(a, b))
        self.assertFalse(torch.equal(before_local, g.get_state()))
        self.assertTrue(torch.equal(global_rng, torch.get_rng_state()))
        for before, after in zip(old, (mu, cov, q)):
            self.assertTrue(torch.equal(before, after))
        for bad in (None, 1, "cpu"):
            with self.assertRaises(TypeError):
                belief.sample(mu, cov, q, 16, 3, bad)

    def test_sampling_dt_keyword_applies_before_first_future_output(self):
        mu, cov = distribution(1)
        a = belief.sample(mu, cov, torch.eye(6, dtype=torch.float64) * .001,
                          8, 1, generator(), dt=.2)
        b = belief.sample(mu, cov, torch.eye(6, dtype=torch.float64) * .001,
                          8, 1, generator(), dt=.1)
        # Identical initial draws/process noise: dt changes only position propagation.
        self.assertTrue(torch.equal(a[..., 3:], b[..., 3:]))
        self.assertFalse(torch.equal(a[..., :3], b[..., :3]))

    def test_cholesky_retries_are_per_environment_and_recorded(self):
        mu, cov = distribution(2)
        cov[0].zero_()
        q = torch.zeros(2, 6, 6, dtype=torch.float64)
        q[1] = torch.eye(6, dtype=torch.float64) * .01
        paths, diag = belief.sample(mu, cov, q, 8, 2, generator(), return_diagnostics=True)
        self.assertTrue(bool(torch.isfinite(paths).all()))
        self.assertEqual(diag["initial_cholesky"]["retries"].tolist(), [1, 0])
        self.assertEqual(diag["process_cholesky"]["retries"].tolist(), [1, 0])
        self.assertEqual(diag["initial_cholesky"]["jitter"].tolist(), [1e-12, 0.])
        with self.assertRaises(ValueError):
            belief.sample(mu, cov, q, 8, 2, generator(), max_retries=0)

    def test_covariance_and_probability_validation_fail_closed(self):
        pred, R = update_fixture()
        p = torch.ones(2, 3, 4, dtype=torch.float64)
        for bad in (-.01, 1.01, float("nan")):
            q = p.clone()
            q[0, 0, 0] = bad
            with self.assertRaises(ValueError):
                belief.expected_cov(pred, q, R)
        bad_cov = pred.clone()
        bad_cov[0, 0, 0, 0, 0] = -1.
        with self.assertRaises(ValueError):
            belief.expected_cov(bad_cov, p, R)
        with self.assertRaises(ValueError):
            belief.expected_cov(pred, p, R[:, :2, :2])
        with self.assertRaises(ValueError):
            belief.expected_cov(torch.zeros_like(pred), p, torch.zeros_like(R))
        mu, cov = distribution()
        cov[0, 0, 1] += .1
        with self.assertRaises(ValueError):
            belief.predict(mu, cov, .1, torch.eye(6, dtype=torch.float64), 1)

    def test_float32_path_and_invalid_shapes_counts(self):
        mu, cov = distribution()
        mu, cov = mu.float(), cov.float()
        q = belief.tracker_Q(.1, dtype=torch.float32)
        means, predicted = belief.predict(mu, cov, .1, q, 2)
        self.assertEqual(means.dtype, torch.float32)
        self.assertTrue(bool(torch.isfinite(predicted).all()))
        self.assertEqual(belief.sample(mu, cov, q, 4, 2, generator()).dtype, torch.float32)
        for dt in (0., -.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                belief.tracker_Q(dt)
        for count in (0, -1, True):
            with self.assertRaises(ValueError):
                belief.predict(mu, cov, .1, q, count)
        with self.assertRaises(ValueError):
            belief.predict(mu[:, :3], cov, .1, q, 2)
        with self.assertRaises(ValueError):
            belief.predict(mu, cov, .1, torch.eye(3), 2)


if __name__ == "__main__":
    unittest.main()
