"""
A2C PPO continuous agent identical to rl_games but with reward-stability early stop.

Copied from rl_games ``ContinuousA2CBase.train`` (near line 1360 in a2c_common.py) with one
inject block marked ``aerial_early_stop``. If rl_games is upgraded and training breaks here,
refresh this ``train()`` from the upstream method.
"""

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
    def restore(self, fn, set_epoch=True):
        """Restore weights/state, but never inherit a stale checkpoint learning rate."""
        super().restore(fn, set_epoch=set_epoch)
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
