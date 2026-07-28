"""
A2C PPO continuous agent identical to rl_games but with reward-stability early stop.

Copied from rl_games ``ContinuousA2CBase.train`` (near line 1360 in a2c_common.py) with one
inject block marked ``aerial_early_stop``. If rl_games is upgraded and training breaks here,
refresh this ``train()`` from the upstream method.
"""

import math
import os
import re
import time

import numpy as np
import torch
import torch.distributed as dist

from rl_games.algos_torch import torch_ext
from rl_games.algos_torch.a2c_continuous import A2CAgent

from aerial_gym.rl_training.rl_games.reward_stable_early_stop import (
    collapse_from_peak_should_stop,
    parse_early_stop_collapse_config,
    parse_early_stop_stable_config,
    window_band_stable_should_stop,
)
from aerial_gym.rl_training.rl_games.aerial_tensorboard import write_aerial_epoch_scalars
from aerial_gym.rl_training.rl_games.pretty_train_stats import print_training_dashboard
from aerial_gym.rl_training.rl_games.ppo_update_safety import (
    lateral_latent_margin_loss,
    stable_ppo_actor_loss,
)
from aerial_gym.rl_training.rl_games.train_run_recorder import (
    finalize_agent_run,
    record_agent_epoch,
)
from aerial_gym.rl_training.rl_games.training_safety import (
    first_nonfinite_training_value,
    is_finite_training_value,
    reset_optimizer_learning_rate,
)
from aerial_gym.task.position_setpoint_task.train_dashboard import consume_epoch_intercept_summary


_AERIAL_FINISHED_MARKER = ".aerial_training_finished"
_CKPT_REWARD_RE = re.compile(r"_rew_([-+]?\d+(?:\.\d+)?)")
_FAILED_EXIT_REASONS = {"early_stop_nan", "nonfinite_ppo"}


def _scalar_mean_reward(mean_r0):
    """rl-games may return a CUDA tensor, CPU tensor, or NumPy scalar from game_rewards averages."""
    if isinstance(mean_r0, torch.Tensor):
        return float(mean_r0.detach().cpu().item())
    if isinstance(mean_r0, np.generic):
        return float(mean_r0)
    return float(mean_r0)


def _resolve_num_parallel_envs(agent) -> int:
    import os

    cfg = getattr(agent, "config", None) or {}
    env_cfg = cfg.get("env_config") if isinstance(cfg, dict) else None
    candidates = (
        cfg.get("num_actors") if isinstance(cfg, dict) else None,
        env_cfg.get("num_envs") if isinstance(env_cfg, dict) else None,
        os.environ.get("NUM_ENVS"),
        getattr(agent, "num_agents", None),
    )
    for raw in candidates:
        if raw is None or str(raw).strip() == "":
            continue
        try:
            v = int(raw)
            if v > 1:
                return v
        except (TypeError, ValueError):
            continue
    return 128


def _read_existing_best_reward(nn_dir: str) -> float:
    """Prevent resume runs from overwriting a better gen_ppo*.pth with a lower current policy."""
    run_root = os.path.dirname(os.path.abspath(nn_dir))
    best = -1000000000.0

    csv_path = os.path.join(run_root, "aerial_run", "epoch_metrics.csv")
    try:
        import csv

        with open(csv_path, "r", encoding="utf-8", newline="") as f:
            for row in csv.DictReader(f):
                raw = row.get("mean_reward")
                if raw in (None, ""):
                    continue
                best = max(best, float(raw))
    except (OSError, ValueError):
        pass

    try:
        for name in os.listdir(nn_dir):
            if not name.endswith(".pth"):
                continue
            match = _CKPT_REWARD_RE.search(name)
            if match:
                best = max(best, float(match.group(1)))
    except (OSError, ValueError):
        pass

    return best


class EarlyStopA2CAgent(A2CAgent):
    def calc_gradients(self, input_dict):
        """rl-games PPO update with opt-in ratio, KL and lateral-mean safety controls."""
        value_preds_batch = input_dict["old_values"]
        old_action_log_probs_batch = input_dict["old_logp_actions"]
        advantage = input_dict["advantages"]
        old_mu_batch = input_dict["mu"]
        old_sigma_batch = input_dict["sigma"]
        return_batch = input_dict["returns"]
        actions_batch = input_dict["actions"]
        obs_batch = self._preproc_obs(input_dict["obs"])

        lr_mul = 1.0
        curr_e_clip = self.e_clip
        batch_dict = {
            "is_train": True,
            "prev_actions": actions_batch,
            "obs": obs_batch,
        }

        rnn_masks = None
        if self.is_rnn:
            rnn_masks = input_dict["rnn_masks"]
            batch_dict["rnn_states"] = input_dict["rnn_states"]
            batch_dict["seq_length"] = self.seq_length
            if self.zero_rnn_on_done:
                batch_dict["dones"] = input_dict["dones"]

        with torch.amp.autocast(
            "cuda", enabled=self.mixed_precision, dtype=torch.bfloat16
        ):
            res_dict = self.model(batch_dict)
            action_log_probs = res_dict["prev_neglogp"]
            values = res_dict["values"]
            entropy = res_dict["entropy"]
            mu = res_dict["mus"]
            sigma = res_dict["sigmas"]

            with torch.no_grad():
                reduce_kl = rnn_masks is None
                kl_dist = torch_ext.policy_kl(
                    mu.detach(),
                    sigma.detach(),
                    old_mu_batch,
                    old_sigma_batch,
                    reduce_kl,
                )
                if rnn_masks is not None:
                    kl_dist = (kl_dist * rnn_masks).sum() / rnn_masks.numel()

            raw_kl_gate = os.environ.get("NAVRL_PPO_KL_STOP", "").strip()
            kl_gate = float(raw_kl_gate) if raw_kl_gate else 0.0
            if not math.isfinite(kl_gate) or kl_gate < 0.0:
                raise ValueError("NAVRL_PPO_KL_STOP must be finite and >= 0")

            # Do not let different DDP ranks make different backward/skip decisions. NavRL uses one
            # GPU; multi-GPU callers retain the stock synchronized update path.
            skip_for_kl = (
                not self.multi_gpu
                and kl_gate > 0.0
                and bool(torch.isfinite(kl_dist))
                and float(kl_dist.detach().cpu()) > kl_gate
            )
            if skip_for_kl:
                zero = action_log_probs.detach().new_zeros(())
                self.aux_loss_dict = {}
                self._aerial_kl_skipped_minibatches = (
                    int(getattr(self, "_aerial_kl_skipped_minibatches", 0)) + 1
                )
                skipped = self._aerial_kl_skipped_minibatches
                if skipped == 1 or skipped % 50 == 0:
                    print(
                        "[aerial RL] PPO minibatch skipped by KL gate | "
                        f"kl={float(kl_dist):.5f} > {kl_gate:.5f} "
                        f"(total skipped={skipped}).",
                        flush=True,
                    )
                self.diagnostics.mini_batch(
                    self,
                    {
                        "values": value_preds_batch,
                        "returns": return_batch,
                        "new_neglogp": action_log_probs,
                        "old_neglogp": old_action_log_probs_batch,
                        "masks": rnn_masks,
                    },
                    curr_e_clip,
                    0,
                )
                self.train_result = (
                    zero,
                    zero,
                    entropy.detach().mean(),
                    kl_dist,
                    self.last_lr,
                    lr_mul,
                    mu.detach(),
                    sigma.detach(),
                    zero,
                )
                return

            raw_log_ratio_clamp = os.environ.get(
                "NAVRL_PPO_LOG_RATIO_CLAMP", ""
            ).strip()
            log_ratio_clamp = (
                float(raw_log_ratio_clamp) if raw_log_ratio_clamp else 0.0
            )
            if not math.isfinite(log_ratio_clamp) or log_ratio_clamp < 0.0:
                raise ValueError(
                    "NAVRL_PPO_LOG_RATIO_CLAMP must be finite and >= 0"
                )
            actor_loss_func = self.actor_loss_func
            if log_ratio_clamp > 0.0:
                actor_loss_func = lambda old_nlp, new_nlp, adv, is_ppo, e_clip: (
                    stable_ppo_actor_loss(
                        old_nlp,
                        new_nlp,
                        adv,
                        is_ppo,
                        e_clip,
                        log_ratio_clamp,
                    )
                )

            loss, a_loss, c_loss, entropy, b_loss, sum_mask = self.calc_losses(
                actor_loss_func,
                old_action_log_probs_batch,
                action_log_probs,
                advantage,
                curr_e_clip,
                value_preds_batch,
                values,
                return_batch,
                mu,
                entropy,
                rnn_masks,
            )

            aux_loss = self.model.get_aux_loss()
            self.aux_loss_dict = {}
            if aux_loss is not None:
                for key, value in aux_loss.items():
                    loss += value
                    if key in self.aux_loss_dict:
                        self.aux_loss_dict[key] = value.detach()
                    else:
                        self.aux_loss_dict[key] = [value.detach()]

            raw_margin = os.environ.get("NAVRL_LATENT_MARGIN_Y", "").strip()
            raw_margin_coef = os.environ.get(
                "NAVRL_LATENT_MARGIN_COEF", ""
            ).strip()
            margin = float(raw_margin) if raw_margin else 0.0
            margin_coef = float(raw_margin_coef) if raw_margin_coef else 0.0
            if (
                not math.isfinite(margin)
                or not math.isfinite(margin_coef)
                or margin < 0.0
                or margin_coef < 0.0
            ):
                raise ValueError(
                    "NAVRL_LATENT_MARGIN_Y and NAVRL_LATENT_MARGIN_COEF "
                    "must be finite and >= 0"
                )
            if margin > 0.0 and margin_coef > 0.0:
                margin_penalty = lateral_latent_margin_loss(mu, margin)
                weighted_margin = margin_coef * margin_penalty
                loss += weighted_margin
                self.aux_loss_dict["lateral_latent_margin"] = [
                    weighted_margin.detach()
                ]

            if self.multi_gpu:
                self.optimizer.zero_grad(set_to_none=True)
            else:
                for parameter in self.model.parameters():
                    parameter.grad = None

        self.scaler.scale(loss).backward()
        self.trancate_gradients_and_step()

        self.diagnostics.mini_batch(
            self,
            {
                "values": value_preds_batch,
                "returns": return_batch,
                "new_neglogp": action_log_probs,
                "old_neglogp": old_action_log_probs_batch,
                "masks": rnn_masks,
            },
            curr_e_clip,
            0,
        )
        self.train_result = (
            a_loss,
            c_loss,
            entropy,
            kl_dist,
            self.last_lr,
            lr_mul,
            mu.detach(),
            sigma.detach(),
            b_loss,
        )

    def get_action_values(self, obs):
        """Collect policy-output diagnostics before rl_games clips actions for the environment."""
        result = super().get_action_values(obs)
        if os.environ.get("NAVRL_ACTION_DIAG", "0").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        ):
            self._accumulate_policy_action_diag(result)
        return result

    def _accumulate_policy_action_diag(self, result):
        actions = result.get("actions")
        if not isinstance(actions, torch.Tensor) or actions.ndim != 2:
            return
        with torch.no_grad():
            actions = actions.detach()
            mus = result.get("mus")
            sigmas = result.get("sigmas")
            diag = getattr(self, "_policy_action_diag", None)
            if not isinstance(diag, dict):
                diag = {
                    "n": 0,
                    "raw_oob": torch.zeros(actions.shape[1], device=actions.device),
                    "edge95": torch.zeros(actions.shape[1], device=actions.device),
                    "edge99": torch.zeros(actions.shape[1], device=actions.device),
                    "abs_sum": torch.zeros(actions.shape[1], device=actions.device),
                    "mu_abs_sum": torch.zeros(actions.shape[1], device=actions.device),
                    "sigma_sum": torch.zeros(actions.shape[1], device=actions.device),
                    "delta_y_sum": 0.0,
                    "delta_y_n": 0,
                }
            finite = torch.isfinite(actions)
            safe = torch.where(finite, actions, torch.zeros_like(actions))
            diag["n"] += int(actions.shape[0])
            diag["raw_oob"] += ((safe.abs() > 1.0) & finite).sum(dim=0)
            diag["edge95"] += ((safe.abs() >= 0.95) & finite).sum(dim=0)
            diag["edge99"] += ((safe.abs() >= 0.99) & finite).sum(dim=0)
            diag["abs_sum"] += (safe.abs() * finite).sum(dim=0)
            if isinstance(mus, torch.Tensor) and mus.shape == actions.shape:
                diag["mu_abs_sum"] += mus.detach().abs().sum(dim=0)
            if isinstance(sigmas, torch.Tensor) and sigmas.shape == actions.shape:
                diag["sigma_sum"] += sigmas.detach().sum(dim=0)

            previous = getattr(self, "_policy_action_diag_prev", None)
            if isinstance(previous, torch.Tensor) and previous.shape == actions.shape:
                valid = finite[:, 1]
                dones = getattr(self, "dones", None)
                if isinstance(dones, torch.Tensor) and dones.shape[0] == actions.shape[0]:
                    valid &= ~dones.bool()
                if bool(valid.any()):
                    diag["delta_y_sum"] += float(
                        (safe[valid, 1] - previous[valid, 1]).abs().sum().item()
                    )
                    diag["delta_y_n"] += int(valid.sum().item())
            self._policy_action_diag_prev = safe.clone()
            self._policy_action_diag = diag

    def _consume_policy_action_diag(self):
        diag = getattr(self, "_policy_action_diag", None)
        self._policy_action_diag = None
        if not isinstance(diag, dict) or int(diag.get("n", 0)) <= 0:
            return None
        n = max(1, int(diag["n"]))
        delta_n = max(1, int(diag["delta_y_n"]))

        def values(key):
            return [float(v) / n for v in diag[key].detach().cpu().tolist()]

        return {
            "raw_oob": values("raw_oob"),
            "edge95": values("edge95"),
            "edge99": values("edge99"),
            "mean_abs": values("abs_sum"),
            "mean_mu_abs": values("mu_abs_sum"),
            "mean_sigma": values("sigma_sum"),
            "delta_y": float(diag["delta_y_sum"]) / delta_n,
            "n": n,
        }

    def restore(self, fn, set_epoch=True):
        """Restore weights/state, but never inherit a stale checkpoint learning rate."""
        super().restore(fn, set_epoch=set_epoch)
        reset_actor_optimizer = os.environ.get(
            "NAVRL_RESET_ACTOR_OPTIMIZER", "0"
        ).strip().lower() in ("1", "true", "yes", "on")
        if reset_actor_optimizer:
            # A bounded-distribution branch reuses the competent representation/mean/value weights
            # but changes the actor likelihood geometry.  Old Adam moments point along the legacy
            # clipped-Gaussian objective and caused a large first update in dry-run checks.
            # Keep the separately owned asymmetric critic optimizer/state; reset only actor PPO.
            self.optimizer.state.clear()
            print(
                "[aerial RL] Actor optimizer moments reset for action-distribution branch; "
                "central critic optimizer retained.",
                flush=True,
            )
        configured_lr = float(self.config["learning_rate"])
        self.last_lr = configured_lr
        reset_optimizer_learning_rate(self.optimizer, configured_lr)
        print(
            "[aerial RL] Resume optimizer LR reset to active config: "
            f"{configured_lr:.3g}.",
            flush=True,
        )

    def write_stats(
        self,
        total_time,
        epoch_num,
        step_time,
        play_time,
        update_time,
        a_losses,
        c_losses,
        entropies,
        kls,
        last_lr,
        lr_mul,
        frame,
        scaled_time,
        scaled_play_time,
        curr_frames,
    ):
        """Disable rl-games verbose TB (/step, /time, performance/*, diagnostics spam)."""
        _ = (
            total_time,
            play_time,
            update_time,
            frame,
            scaled_time,
            scaled_play_time,
        )
        self._aerial_tb_pending = {
            "epoch_num": epoch_num,
            "step_time": step_time,
            "curr_frames": curr_frames,
            "a_losses": a_losses,
            "c_losses": c_losses,
            "entropies": entropies,
            "kls": kls,
            "last_lr": last_lr,
            "lr_mul": lr_mul,
        }

    def _flush_aerial_tensorboard(
        self,
        *,
        mean_reward,
        mean_episode_length,
        intercept_succ,
        intercept_done,
        intercept_envs_hit,
        num_parallel_envs,
        mean_target_dist_m=None,
        mean_closest_target_dist_m=None,
        intercept_extra_metrics=None,
        explained_variance=None,
    ):
        pending = getattr(self, "_aerial_tb_pending", None)
        if pending is None or self.writer is None:
            return
        epoch_num = int(pending["epoch_num"])
        write_aerial_epoch_scalars(
            self.writer,
            epoch_num,
            mean_reward=mean_reward,
            mean_episode_length=mean_episode_length,
            mean_target_dist_m=mean_target_dist_m,
            a_losses=pending["a_losses"],
            c_losses=pending["c_losses"],
            entropies=pending["entropies"],
            kls=pending["kls"],
            last_lr=pending["last_lr"],
            lr_mul=pending["lr_mul"],
            step_time=pending["step_time"],
            curr_frames=pending["curr_frames"],
            intercept_succ=intercept_succ,
            intercept_done=intercept_done,
            intercept_envs_hit=intercept_envs_hit,
            num_parallel_envs=num_parallel_envs,
            mean_closest_target_dist_m=mean_closest_target_dist_m,
            explained_variance=explained_variance,
            extra_intercept_metrics=intercept_extra_metrics,
        )
        action_diag = self._consume_policy_action_diag()
        skipped_total = int(getattr(self, "_aerial_kl_skipped_minibatches", 0))
        skipped_previous = int(getattr(self, "_aerial_kl_skipped_last_epoch", 0))
        self.writer.add_scalar(
            "ppo/kl_skipped_minibatches",
            max(0, skipped_total - skipped_previous),
            epoch_num,
        )
        self._aerial_kl_skipped_last_epoch = skipped_total
        if action_diag is not None:
            axis_names = ("x", "y", "z", "yaw")
            for axis, name in enumerate(axis_names):
                self.writer.add_scalar(
                    f"policy_action/raw_oob_{name}",
                    action_diag["raw_oob"][axis],
                    epoch_num,
                )
                self.writer.add_scalar(
                    f"policy_action/edge95_{name}",
                    action_diag["edge95"][axis],
                    epoch_num,
                )
                self.writer.add_scalar(
                    f"policy_action/edge99_{name}",
                    action_diag["edge99"][axis],
                    epoch_num,
                )
                self.writer.add_scalar(
                    f"policy_action/mean_abs_{name}",
                    action_diag["mean_abs"][axis],
                    epoch_num,
                )
                self.writer.add_scalar(
                    f"policy_action/mean_sigma_{name}",
                    action_diag["mean_sigma"][axis],
                    epoch_num,
                )
                self.writer.add_scalar(
                    f"policy_action/mean_mu_abs_{name}",
                    action_diag["mean_mu_abs"][axis],
                    epoch_num,
                )
            self.writer.add_scalar(
                "policy_action/delta_y", action_diag["delta_y"], epoch_num
            )
            if epoch_num == 1 or epoch_num % 25 == 0:
                print(
                    "[aerial RL] policy-actiondiag | mode=%s raw_oob_y=%.4f "
                    "edge95_y=%.4f edge99_y=%.4f |mu_y|=%.3f sigma_y=%.3f "
                    "delta_y=%.3f (n=%d)"
                    % (
                        os.environ.get("NAVRL_ACTION_POLICY", "legacy"),
                        action_diag["raw_oob"][1],
                        action_diag["edge95"][1],
                        action_diag["edge99"][1],
                        action_diag["mean_mu_abs"][1],
                        action_diag["mean_sigma"][1],
                        action_diag["delta_y"],
                        action_diag["n"],
                    ),
                    flush=True,
                )
        if getattr(self, "last_mean_rewards", None) is not None:
            try:
                self.writer.add_scalar("stability/best_reward", float(self.last_mean_rewards), epoch_num)
            except (TypeError, ValueError):
                pass
        try:
            self.writer.flush()
        except Exception:
            pass

    def _write_aerial_training_finished_marker(self, epoch_num):
        """Mark this run folder so train.sh skips auto-resume after a normal exit (crash has no marker)."""
        try:
            run_root = os.path.dirname(os.path.abspath(self.nn_dir))
            marker = os.path.join(run_root, _AERIAL_FINISHED_MARKER)
            with open(marker, "w", encoding="utf-8") as f:
                f.write(f"epoch={epoch_num}\n")
        except OSError:
            pass

    def train(self):
        self.init_tensors()
        self.last_mean_rewards = _read_existing_best_reward(self.nn_dir)
        if getattr(self, "global_rank", 0) == 0 and self.last_mean_rewards > -999999999:
            print(
                "[aerial RL] Best-checkpoint guard: existing best reward "
                f"{self.last_mean_rewards:.3f}; lower current policies will not overwrite "
                f"{self.config['name']}.pth.",
                flush=True,
            )
        start_time = time.perf_counter()
        total_time = 0
        rep_count = 0
        self.obs = self.env_reset()
        self.curr_frames = self.batch_size_envs

        early_cfg = parse_early_stop_stable_config(self.config.get("early_stop_stable"))
        collapse_cfg = parse_early_stop_collapse_config(self.config.get("early_stop_collapse"))

        if self.multi_gpu:
            torch.cuda.set_device(self.local_rank)
            print("====================broadcasting parameters")
            model_params = [self.model.state_dict()]
            if self.has_central_value:
                model_params.append(self.central_value_net.state_dict())
            dist.broadcast_object_list(model_params, 0)
            self.model.load_state_dict(model_params[0])
            if self.has_central_value:
                self.central_value_net.load_state_dict(model_params[1])
            print("====================broadcast done")

        exit_reason = "unknown"
        try:
            while True:
                epoch_num = self.update_epoch()
                step_time, play_time, update_time, sum_time, a_losses, c_losses, b_losses, entropies, kls, last_lr, lr_mul = (
                    self.train_epoch()
                )
                nonfinite_path = first_nonfinite_training_value(
                    (
                        ("ppo/a_loss", a_losses),
                        ("ppo/c_loss", c_losses),
                        ("ppo/b_loss", b_losses),
                        ("ppo/entropy", entropies),
                        ("ppo/kl", kls),
                    ),
                    self.model.named_parameters(),
                )
                if nonfinite_path is not None:
                    exit_reason = "nonfinite_ppo"
                    raise FloatingPointError(
                        "[aerial RL] Non-finite PPO state detected at "
                        f"epoch {epoch_num}: {nonfinite_path}. "
                        "Optimizer output is discarded; use the last finite checkpoint."
                    )
                total_time += sum_time
                frame = self.frame // self.num_agents

                # cleaning memory to optimize space
                self.dataset.update_values_dict(None)
                should_exit = False
                pending_exit_reason = None

                if self.global_rank == 0:
                    self.diagnostics.epoch(self, current_epoch=epoch_num)
                    scaled_time = self.num_agents * sum_time
                    scaled_play_time = self.num_agents * play_time
                    curr_frames = self.curr_frames * self.world_size if self.multi_gpu else self.curr_frames
                    self.frame += curr_frames

                    dash_mr = dash_el = None
                    dash_dist = None
                    dash_closest_dist = None
                    intercept_extra_metrics = None
                    n_agents = _resolve_num_parallel_envs(self)
                    intercept_lines = None
                    intercept_succ = intercept_done = intercept_envs_hit = 0

                    self.write_stats(
                        total_time,
                        epoch_num,
                        step_time,
                        play_time,
                        update_time,
                        a_losses,
                        c_losses,
                        entropies,
                        kls,
                        last_lr,
                        lr_mul,
                        frame,
                        scaled_time,
                        scaled_play_time,
                        curr_frames,
                    )

                    exp_var = None
                    if getattr(self.diagnostics, "diag_dict", None):
                        exp_var = self.diagnostics.diag_dict.get("diagnostics/exp_var")

                    if self.game_rewards.current_size > 0:
                        mean_rewards = self.game_rewards.get_mean()
                        mean_lengths = self.game_lengths.get_mean()
                        self.mean_rewards = mean_rewards[0]
                        dash_mr = _scalar_mean_reward(mean_rewards[0])
                        if not is_finite_training_value(dash_mr):
                            print(
                                "[aerial RL] Early stop: NaN/Inf mean reward — "
                                f"epoch {epoch_num}. No corrupted checkpoint will be saved."
                            )
                            should_exit = True
                            pending_exit_reason = "early_stop_nan"
                        try:
                            dash_el = _scalar_mean_reward(mean_lengths[0])
                        except (TypeError, IndexError, KeyError):
                            dash_el = _scalar_mean_reward(mean_lengths)

                        if self.has_self_play_config:
                            self.self_play_manager.update(self)

                        checkpoint_name = self.config["name"] + "_ep_" + str(epoch_num) + "_rew_" + str(mean_rewards[0])

                        if not should_exit and early_cfg is not None:
                            mr0 = _scalar_mean_reward(mean_rewards[0])
                            es_state = getattr(self, "_aerial_early_stop_state", None)
                            stop_stable, es_state_next = window_band_stable_should_stop(
                                early_cfg,
                                epoch_num,
                                mr0,
                                es_state,
                            )
                            self._aerial_early_stop_state = es_state_next
                            if stop_stable:
                                tail = "_rew_" + str(mean_rewards).replace("[", "_").replace("]", "_")
                                self.save(
                                    os.path.join(
                                        self.nn_dir,
                                        "last_" + self.config["name"] + "_ep_" + str(epoch_num) + tail,
                                    )
                                )
                                print(
                                    "[aerial RL] Early stop: mean reward converged — "
                                    f"last {early_cfg['consecutive_epochs']} epochs "
                                    f"max−min ≤ {early_cfg['reward_band']} "
                                    f"(actual span {es_state_next.get('last_spread', float('nan')):.2f}, "
                                    f"epoch {epoch_num})."
                                )
                                should_exit = True
                                pending_exit_reason = "early_stop_stable"

                        if not should_exit and collapse_cfg is not None and self.game_rewards.current_size > 0:
                            mr0 = _scalar_mean_reward(mean_rewards[0])
                            cs_state = getattr(self, "_aerial_collapse_stop_state", None)
                            stop_collapse, cs_state_next = collapse_from_peak_should_stop(
                                collapse_cfg,
                                epoch_num,
                                mr0,
                                cs_state,
                            )
                            self._aerial_collapse_stop_state = cs_state_next
                            if stop_collapse:
                                if cs_state_next.get("nan_stop"):
                                    print(
                                        "[aerial RL] Early stop: NaN/Inf mean reward — "
                                        f"epoch {epoch_num}. No corrupted checkpoint will be saved."
                                    )
                                    pending_exit_reason = "early_stop_nan"
                                else:
                                    print(
                                        "[aerial RL] Early stop: reward collapse from peak — "
                                        f"peak {cs_state_next.get('collapse_peak', float('nan')):.1f} → "
                                        f"now {cs_state_next.get('collapse_mr', float('nan')):.1f} "
                                        f"for {collapse_cfg['patience_epochs']} epochs (epoch {epoch_num})."
                                    )
                                    pending_exit_reason = "early_stop_collapse"
                                    tail = "_rew_" + str(mean_rewards).replace("[", "_").replace("]", "_")
                                    self.save(
                                        os.path.join(
                                            self.nn_dir,
                                            "last_"
                                            + self.config["name"]
                                            + "_ep_"
                                            + str(epoch_num)
                                            + tail,
                                        )
                                    )
                                should_exit = True

                        if not should_exit and self.save_freq > 0:
                            if epoch_num % self.save_freq == 0:
                                self.save(os.path.join(self.nn_dir, "last_" + checkpoint_name))

                        current_mean_reward = _scalar_mean_reward(mean_rewards[0])
                        if (
                            not should_exit
                            and current_mean_reward > self.last_mean_rewards
                            and epoch_num >= self.save_best_after
                        ):
                            print("saving next best rewards: ", mean_rewards)
                            self.last_mean_rewards = current_mean_reward
                            self.save(os.path.join(self.nn_dir, self.config["name"]))

                            if "score_to_win" in self.config:
                                if self.last_mean_rewards > self.config["score_to_win"]:
                                    print("Maximum reward achieved. Network won!")
                                    self.save(os.path.join(self.nn_dir, checkpoint_name))
                                    should_exit = True
                                    pending_exit_reason = "score_to_win"

                    try:
                        (
                            intercept_lines,
                            intercept_succ,
                            intercept_done,
                            intercept_envs_hit,
                            n_agents,
                            dash_dist,
                            dash_closest_dist,
                            intercept_extra_metrics,
                        ) = (
                            consume_epoch_intercept_summary(n_agents)
                        )
                    except Exception:
                        pass

                    # NavRL navigation tasks feed their own per-epoch stats; when present they
                    # replace the intercept lines in the dashboard and add navrl/* TB scalars.
                    try:
                        from aerial_gym.task.navrl_task.train_dashboard import (
                            consume_navrl_epoch_summary,
                        )

                        nav_lines, nav_metrics, nav_done = consume_navrl_epoch_summary()
                        if nav_done > 0:
                            intercept_lines = nav_lines
                            if intercept_extra_metrics:
                                intercept_extra_metrics.update(nav_metrics)
                            else:
                                intercept_extra_metrics = nav_metrics
                    except Exception:
                        pass

                    self._flush_aerial_tensorboard(
                        mean_reward=dash_mr,
                        mean_episode_length=dash_el,
                        mean_target_dist_m=dash_dist,
                        mean_closest_target_dist_m=dash_closest_dist,
                        intercept_extra_metrics=intercept_extra_metrics,
                        intercept_succ=intercept_succ,
                        intercept_done=intercept_done,
                        intercept_envs_hit=intercept_envs_hit,
                        num_parallel_envs=n_agents,
                        explained_variance=exp_var,
                    )

                    print_training_dashboard(
                        self.print_stats,
                        curr_frames,
                        step_time,
                        scaled_play_time,
                        scaled_time,
                        epoch_num,
                        self.max_epochs,
                        frame,
                        self.max_frames,
                        mean_reward=dash_mr,
                        mean_episode_length=dash_el,
                        intercept_lines=intercept_lines,
                    )

                    record_agent_epoch(
                        self,
                        epoch_num=epoch_num,
                        mean_reward=dash_mr,
                        mean_episode_length=dash_el,
                        mean_target_dist_m=dash_dist,
                        mean_closest_target_dist_m=dash_closest_dist,
                        intercept_extra_metrics=intercept_extra_metrics,
                        intercept_succ=intercept_succ,
                        intercept_done=intercept_done,
                        intercept_envs_hit=intercept_envs_hit,
                        num_parallel_envs=n_agents,
                    )

                    if (
                        not should_exit
                        and epoch_num >= self.max_epochs
                        and self.max_epochs != -1
                    ):
                        if self.game_rewards.current_size == 0:
                            print("WARNING: Max epochs reached before any env terminated at least once")
                            mean_rewards = -np.inf

                        self.save(
                            os.path.join(
                                self.nn_dir,
                                "last_" + self.config["name"] + "_ep_" + str(epoch_num)
                                + "_rew_" + str(mean_rewards).replace("[", "_").replace("]", "_"),
                            )
                        )
                        print("MAX EPOCHS NUM!")
                        should_exit = True
                        pending_exit_reason = "max_epochs"

                    if (
                        not should_exit
                        and self.frame >= self.max_frames
                        and self.max_frames != -1
                    ):
                        if self.game_rewards.current_size == 0:
                            print("WARNING: Max frames reached before any env terminated at least once")
                            mean_rewards = -np.inf

                        self.save(
                            os.path.join(
                                self.nn_dir,
                                "last_" + self.config["name"] + "_frame_" + str(self.frame)
                                + "_rew_" + str(mean_rewards).replace("[", "_").replace("]", "_"),
                            )
                        )
                        print("MAX FRAMES NUM!")
                        should_exit = True
                        pending_exit_reason = "max_frames"

                if self.multi_gpu:
                    should_exit_t = torch.tensor(should_exit, device=self.device).float()
                    dist.broadcast(should_exit_t, 0)
                    should_exit = bool(should_exit_t.item())

                if should_exit:
                    exit_reason = pending_exit_reason or "completed"
                    if (
                        exit_reason not in _FAILED_EXIT_REASONS
                        and getattr(self, "global_rank", 0) == 0
                    ):
                        self._write_aerial_training_finished_marker(epoch_num)
                    if exit_reason in _FAILED_EXIT_REASONS:
                        raise FloatingPointError(
                            f"[aerial RL] Training failed with {exit_reason} at epoch {epoch_num}."
                        )
                    break

            return self.last_mean_rewards, epoch_num
        except FloatingPointError:
            if exit_reason == "unknown":
                exit_reason = "nonfinite_ppo"
            raise
        except KeyboardInterrupt:
            exit_reason = "interrupted"
            print("\n[aerial RL] 학습 중단 (KeyboardInterrupt)", flush=True)
            raise
        finally:
            try:
                finalize_agent_run(self, exit_reason)
            except Exception as exc:
                print(f"[aerial RL] run summary skipped: {exc}", flush=True)
