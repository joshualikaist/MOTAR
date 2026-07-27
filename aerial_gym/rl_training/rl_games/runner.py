import numpy as np
import os
import sys
import yaml

from pathlib import Path
from datetime import datetime
from typing import Optional

from aerial_gym.rl_training.rl_games.quiet_rl_io import (
    devnull_stdio,
    install_quiet_print_filter,
    mute_info_logging,
    patch_rlgames_quiet_train_init,
    quiet_startup_enabled,
)
_real_print = __import__("builtins").print

if quiet_startup_enabled():
    with devnull_stdio():
        import isaacgym

        from aerial_gym.registry.task_registry import task_registry
        from aerial_gym.utils.helpers import parse_arguments
else:
    import isaacgym

    from aerial_gym.registry.task_registry import task_registry
    from aerial_gym.utils.helpers import parse_arguments

import gym
from gym import spaces
from argparse import Namespace

from rl_games.common import env_configurations, vecenv

import torch
import distutils

os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
# import warnings
# warnings.filterwarnings("error")

# Vision mode emits obs['states'] for the asymmetric critic during TRAINING. The rl_games PLAYER
# expects a plain actor-obs tensor (observation_space is a Box, not a Dict), so returning the dict
# at play time makes norm_obs choke ("Can't create empty tensor"). Detect --play and hand the
# player only the actor observation; the critic/states are a train-only concern.
_PLAY_MODE = ("--play" in sys.argv) or ("-p" in sys.argv)


class ExtractObsWrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)

    @staticmethod
    def _extract(observations):
        # Asymmetric actor-critic (navrl vision mode): during TRAINING the task also emits
        # privileged critic input under 'states'. rl_games' obs_to_tensors keeps a dict containing
        # an 'obs' key as-is, and get_action_values feeds obs['states'] to the central value net.
        # At PLAY the player wants a plain actor-obs tensor, so drop 'states' there (_PLAY_MODE).
        if "states" in observations and not _PLAY_MODE:
            return {"obs": observations["observations"], "states": observations["states"]}
        return observations["observations"]

    def reset(self, **kwargs):
        observations, *_ = super().reset(**kwargs)
        return self._extract(observations)

    def step(self, action):
        observations, rewards, terminated, truncated, infos = super().step(action)

        dones = torch.where(
            terminated | truncated,
            torch.ones_like(terminated),
            torch.zeros_like(terminated),
        )

        return (
            self._extract(observations),
            rewards,
            dones,
            infos,
        )

    def render(self, mode="human", **kwargs):
        # Tasks vary: some implement render(), some render(mode=...)
        try:
            return self.env.render(mode=mode, **kwargs)
        except TypeError:
            return self.env.render()


class AERIALRLGPUEnv(vecenv.IVecEnv):
    def __init__(self, config_name, num_actors, **kwargs):
        self.env = env_configurations.configurations[config_name]["env_creator"](**kwargs)
        self.env = ExtractObsWrapper(self.env)

    def step(self, actions):
        return self.env.step(actions)

    def reset(self):
        return self.env.reset()

    def get_env_state(self):
        # rl_games saves this into the checkpoint ('env_state'); forward to the task (via the
        # gym.Wrapper) so per-task state like navrl_task's curriculum counter survives resume.
        fn = getattr(self.env, "get_env_state", None)
        return fn() if callable(fn) else None

    def set_env_state(self, state):
        fn = getattr(self.env, "set_env_state", None)
        if callable(fn) and state is not None:
            fn(state)

    def render(self, mode="human"):
        """rl_games BasePlayer expects vecenv.render(mode=...) during play."""
        if hasattr(self.env, "render"):
            return self.env.render(mode=mode)
        return None

    def get_number_of_agents(self):
        return self.env.get_number_of_agents()

    def get_env_info(self):
        info = {}
        info["action_space"] = spaces.Box(
            -np.ones(self.env.task_config.action_space_dim),
            np.ones(self.env.task_config.action_space_dim),
        )
        info["observation_space"] = spaces.Box(
            np.ones(self.env.task_config.observation_space_dim) * -np.Inf,
            np.ones(self.env.task_config.observation_space_dim) * np.Inf,
        )
        # navrl vision mode: expose the privileged-critic state space so rl_games'
        # central_value_config builds against the right shape (falls back to observation_space
        # if absent, which would silently defeat the asymmetric critic).
        state_dim = int(getattr(self.env.task_config, "state_space_dim", 0) or 0)
        if state_dim > 0:
            info["state_space"] = spaces.Box(
                np.ones(state_dim) * -np.Inf, np.ones(state_dim) * np.Inf
            )
        if not quiet_startup_enabled():
            print(info["action_space"], info["observation_space"])
        return info


env_configurations.register(
    "position_setpoint_task",
    {
        "env_creator": lambda **kwargs: task_registry.make_task("position_setpoint_task", **kwargs),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "position_setpoint_task_sim2real",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "position_setpoint_task_sim2real", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "position_setpoint_task_sim2real_px4",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "position_setpoint_task_sim2real_px4", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "position_setpoint_task_acceleration_sim2real",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "position_setpoint_task_acceleration_sim2real", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "navigation_task",
    {
        "env_creator": lambda **kwargs: task_registry.make_task("navigation_task", **kwargs),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "position_setpoint_task_reconfigurable",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "position_setpoint_task_reconfigurable", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "position_setpoint_task_morphy",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "position_setpoint_task_morphy", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "position_setpoint_task_sim2real_end_to_end",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "position_setpoint_task_sim2real_end_to_end", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "shooting_moving_target_task",
    {
        "env_creator": lambda **kwargs: task_registry.make_task(
            "shooting_moving_target_task", **kwargs
        ),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

env_configurations.register(
    "navrl_task",
    {
        "env_creator": lambda **kwargs: task_registry.make_task("navrl_task", **kwargs),
        "vecenv_type": "AERIAL-RLGPU",
    },
)

vecenv.register(
    "AERIAL-RLGPU",
    lambda config_name, num_actors, **kwargs: AERIALRLGPUEnv(config_name, num_actors, **kwargs),
)


def get_args():
    from isaacgym import gymutil

    custom_parameters = [
        {
            "name": "--seed",
            "type": int,
            "default": 0,
            "required": False,
            "help": "Random seed, if larger than 0 will overwrite the value in yaml config.",
        },
        {
            "name": "--tf",
            "required": False,
            "help": "run tensorflow runner",
            "action": "store_true",
        },
        {
            "name": "--train",
            "required": False,
            "help": "train network",
            "action": "store_true",
        },
        {
            "name": "--play",
            "required": False,
            "help": "play(test) network",
            "action": "store_true",
        },
        {
            "name": "--eval",
            "required": False,
            "help": "same as --play (checkpoint inference; avoids accidental TRAIN branch)",
            "action": "store_true",
        },
        {
            "name": "--checkpoint",
            "type": str,
            "required": False,
            "help": "path to checkpoint",
        },
        {
            "name": "--resume_in_place",
            "action": "store_true",
            "default": False,
            "help": "Reuse the checkpoint's run folder even if that run is already marked "
            "finished. By default, checkpoints from finished runs warm-start a new run folder.",
        },
        {
            "name": "--branch_run",
            "action": "store_true",
            "default": False,
            "help": "Warm-start into a new run folder even when the source run was interrupted. "
            "Use this when changing the environment/config so the source metrics stay clean.",
        },
        {
            "name": "--disable_collapse_early_stop",
            "action": "store_true",
            "default": False,
            "help": "Disable only the reward-drop-from-peak early-stop rule for this run. "
            "NaN/Inf fail-fast remains enabled.",
        },
        {
            "name": "--file",
            "type": str,
            "default": "ppo_aerial_quad.yaml",
            "required": False,
            "help": "path to config",
        },
        {
            "name": "--num_envs",
            "type": int,
            "default": "1024",
            "help": "Number of environments to create. Overrides config file if provided.",
        },
        {
            "name": "--sigma",
            "type": float,
            "required": False,
            "help": "sets new sigma value in case if 'fixed_sigma: True' in yaml config",
        },
        {
            "name": "--track",
            "action": "store_true",
            "help": "if toggled, this experiment will be tracked with Weights and Biases",
        },
        {
            "name": "--wandb-project-name",
            "type": str,
            "default": "rl_games",
            "help": "the wandb's project name",
        },
        {
            "name": "--wandb-entity",
            "type": str,
            "default": None,
            "help": "the entity (team) of wandb's project",
        },
        {
            "name": "--task",
            "type": str,
            "default": "navigation_task",
            "help": "Override task from config file if provided.",
        },
        {
            "name": "--experiment_name",
            "type": str,
            "help": "Name of the experiment to run or load. Overrides config file if provided.",
        },
        {
            "name": "--headless",
            "type": lambda x: bool(distutils.util.strtobool(x)),
            "default": "False",
            "help": "Force display off at all times",
        },
        {
            "name": "--horovod",
            "action": "store_true",
            "default": False,
            "help": "Use horovod for multi-gpu training",
        },
        {
            "name": "--rl_device",
            "type": str,
            "default": "cuda:0",
            "help": "Device used by the RL algorithm, (cpu, gpu, cuda:0, cuda:1 etc..)",
        },
        {
            "name": "--use_warp",
            "type": lambda x: bool(distutils.util.strtobool(x)),
            "default": "True",
            "help": "Choose whether to use warp or Isaac Gym rendeing pipeline.",
        },
        {
            "name": "--max_epochs",
            "type": int,
            "default": -1,
            "help": "Override max_epochs in the yaml (> 0). Needed to EXTEND a warm-start: a "
            "--checkpoint resume restores the epoch counter, so a Phase-B run must raise this "
            "above the checkpoint's epoch (e.g. Phase A ended at 6000 -> --max_epochs 12000).",
        },
    ]

    # parse arguments
    args = parse_arguments(description="RL Policy", custom_parameters=custom_parameters)

    # name allignment
    args.sim_device_id = args.compute_device_id
    args.sim_device = args.sim_device_type
    if args.sim_device == "cuda":
        args.sim_device += f":{args.sim_device_id}"
    return args


def _prepare_rlgames_argv_dict(cli: dict):
    """rl_games TorchRunner treats both train=False and play=False as TRAIN — easy to misuse."""
    cli.setdefault("train", False)
    cli.setdefault("play", False)
    cli.setdefault("checkpoint", None)
    cli.setdefault("eval", False)
    cli.setdefault("track", False)
    cli["train"] = bool(cli["train"])
    cli["play"] = bool(cli["play"])
    if cli.pop("eval", False):
        cli["play"] = True
    if cli["train"] and cli["play"]:
        raise SystemExit("[aerial RL] --train 과 --eval/--play 은 동시에 쓸 수 없습니다.")

    ck = cli.get("checkpoint")
    ck_set = ck is not None and str(ck).strip() != ""

    if not cli["train"] and not cli["play"]:
        if ck_set and not quiet_startup_enabled():
            print(
                "[aerial RL WARNING] --checkpoint 만 있고 --train / --play / --eval 이 없습니다.\n"
                "  rl-games 기본값은 학습 재개(TRAIN, TensorBoard 에 epoch 기록)입니다.\n"
                "  재생만 하려면:  --play 또는 --checkpoint 를 쓸 때 같은 줄에 반드시 --play (별칭 --eval)\n"
                "  학습 재개 의도면:  --train 명시를 권장합니다."
            )
        if not quiet_startup_enabled():
            print(
                "[aerial RL] Mode = TRAIN (implicit — TorchRunner 의 default)\n"
                "            추론만: runner.py … --play --checkpoint PATH   (별칭: --eval)"
            )
    elif cli["play"]:
        if not quiet_startup_enabled():
            print("[aerial RL] Mode = PLAY (inference, no PPO 학습 루프·TensorBoard epoch)")
            if not ck_set:
                print("[aerial RL WARNING] --play 인데 체크포인트 없음 → 가중치는 초기 상태입니다.")
    elif not quiet_startup_enabled():
        print("[aerial RL] Mode = TRAIN (--train 또는 기본 학습 재개 동작)")
    return cli


def _run_root_from_checkpoint(ckpt_path, base_dir=None):
    """Resolve runs/<run_name>/ from .../runs/<run_name>/nn/<file>.pth."""
    if ckpt_path is None:
        return None
    s = str(ckpt_path).strip()
    if not s:
        return None
    base = Path(base_dir or os.getcwd()).resolve()
    p = Path(s)
    if not p.is_absolute():
        p = (base / p).resolve()
    else:
        p = p.resolve()
    if p.parent.name != "nn":
        return None
    run_root = p.parent.parent
    if run_root.parent.name == "runs":
        return run_root
    return None


def _run_name_from_checkpoint(ckpt_path, base_dir=None):
    run_root = _run_root_from_checkpoint(ckpt_path, base_dir=base_dir)
    return run_root.name if run_root is not None else None


def _task_run_suffix(task: Optional[str]) -> str:
    """Short label appended to run folder names (fixed vs moving intercept, etc.)."""
    if not task:
        return ""
    mapping = {
        "position_setpoint_task": "fixed",
        "shooting_moving_target_task": "moving",
        "navigation_task": "nav",
        "navrl_task": "navrl",
    }
    if task in mapping:
        return mapping[task]
    # Fallback: strip common _task suffix for readability.
    if task.endswith("_task"):
        return task[: -len("_task")].replace("_", "-")
    return task.replace("_", "-")


def _new_experiment_name(task: Optional[str]) -> str:
    """runs/ folder name: ppo_YYMMDD_HHMM_<fixed|moving|…>."""
    stamp = datetime.now().strftime("ppo_%y%m%d_%H%M")
    suffix = _task_run_suffix(task)
    return f"{stamp}_{suffix}" if suffix else stamp


def _normalize_vf_checkpoint(path):
    """Warm-start fix: torch.compile prefixes the CENTRAL-VALUE (critic) state_dict keys with
    '_orig_mod.' when the checkpoint is saved, but the freshly-built central_value_net at restore
    expects no prefix -> set_full_state_weights' strict load_state_dict('assymetric_vf_nets')
    raises on a `--checkpoint --train` resume of a vision (asymmetric-critic) run. Strip the prefix
    into a sibling *_rlnorm.pth and return that path (or the original if there is nothing to fix).
    The actor 'model' keys are left untouched -- rl_games restores those fine."""
    if not path or not os.path.isfile(path):
        return path
    try:
        ck = torch.load(path, map_location="cpu", weights_only=False)
    except Exception:
        return path
    cv = ck.get("assymetric_vf_nets")
    if not isinstance(cv, dict) or not any("_orig_mod." in k for k in cv):
        return path
    ck["assymetric_vf_nets"] = {k.replace("_orig_mod.", ""): v for k, v in cv.items()}
    out = (path[:-4] if path.endswith(".pth") else path) + "_rlnorm.pth"
    torch.save(ck, out)
    if not quiet_startup_enabled():
        print(f"[aerial RL] normalized central-value checkpoint keys -> {out}")
    return out


def update_config(config, args):

    if args.get("max_epochs", -1) and int(args.get("max_epochs", -1)) > 0:
        config["params"]["config"]["max_epochs"] = int(args["max_epochs"])

    if args.get("disable_collapse_early_stop"):
        train_cfg = config["params"]["config"]
        collapse_cfg = train_cfg.get("early_stop_collapse")
        collapse_cfg = dict(collapse_cfg) if isinstance(collapse_cfg, dict) else {}
        collapse_cfg["enable"] = False
        collapse_cfg["disabled_by"] = "--disable_collapse_early_stop"
        train_cfg["early_stop_collapse"] = collapse_cfg

    if args.get("task") is not None:
        config["params"]["config"]["env_name"] = args["task"]
    if args.get("experiment_name") is not None:
        config["params"]["config"]["name"] = args["experiment_name"]
    config["params"]["config"]["env_config"]["headless"] = args["headless"]
    config["params"]["config"]["env_config"]["num_envs"] = args["num_envs"]
    config["params"]["config"]["env_config"]["use_warp"] = args["use_warp"]
    if args["num_envs"] > 0:
        config["params"]["config"]["num_actors"] = args["num_envs"]
        # config['params']['config']['num_envs'] = args['num_envs']
        config["params"]["config"]["env_config"]["num_envs"] = args["num_envs"]
    if args["seed"] > 0:
        config["params"]["seed"] = args["seed"]
        config["params"]["config"]["env_config"]["seed"] = args["seed"]

    player_cfg = config["params"]["config"].get("player")
    if not isinstance(player_cfg, dict):
        player_cfg = {}
    player_cfg = dict(player_cfg)
    player_cfg["use_vecenv"] = True
    if args.get("play"):
        # Task 쪽 PLAY 대시보드만 쓰고 rl-games 기본 플레이 로그는 끈다.
        player_cfg["print_stats"] = False
        try:
            _gn = int(os.environ.get("PLAY_GAMES_NUM", "64"))
        except ValueError:
            _gn = 64
        player_cfg["games_num"] = max(1, _gn)
    config["params"]["config"]["player"] = player_cfg

    network_override = os.environ.get("NAVRL_NETWORK_OVERRIDE", "").strip()
    if network_override:
        config["params"]["network"]["name"] = network_override

    # Runs folder: ppo_YYMMDD_HHMM_<fixed|moving|…> (local time).
    # Interrupted runs have no finished marker and resume in place. A checkpoint from a completed
    # run is a warm-start branch: putting it back into the source folder mixes old/future epochs,
    # TensorBoard events and checkpoint names, so it gets a new folder by default. PLAY is
    # read-only and may keep the source name; --resume_in_place is the explicit escape hatch.
    _cfg = config["params"]["config"]
    fe = _cfg.get("full_experiment_name", None)
    if isinstance(fe, str) and fe.strip():
        pass
    else:
        resumed_root = _run_root_from_checkpoint(args.get("checkpoint"))
        source_finished = bool(
            resumed_root is not None
            and (resumed_root / ".aerial_training_finished").is_file()
        )
        reuse_source = bool(
            resumed_root is not None
            and not args.get("branch_run")
            and (
                args.get("play")
                or args.get("resume_in_place")
                or not source_finished
            )
        )
        if reuse_source:
            _cfg["full_experiment_name"] = resumed_root.name
        else:
            task = args.get("task") or _cfg.get("env_name")
            _cfg["full_experiment_name"] = _new_experiment_name(task)
            if resumed_root is not None and not quiet_startup_enabled():
                print(
                    "[aerial RL] warm-starting a new run folder "
                    f"({_cfg['full_experiment_name']}). Use --resume_in_place to override."
                )

    return config


if __name__ == "__main__":
    os.makedirs("nn", exist_ok=True)
    os.makedirs("runs", exist_ok=True)

    install_quiet_print_filter()
    mute_info_logging()

    args = vars(get_args())
    args = _prepare_rlgames_argv_dict(args)

    # Warm-start (--checkpoint --train) of a vision run needs the central-value keys normalized
    # (torch.compile '_orig_mod.' prefix); rewrite the path to the normalized copy before restore.
    if args.get("checkpoint") and not args.get("play"):
        args["checkpoint"] = _normalize_vf_checkpoint(args["checkpoint"])

    config_name = args["file"]

    if not quiet_startup_enabled():
        print("Loading config: ", config_name)
    with open(config_name, "r") as stream:
        config = yaml.safe_load(stream)

        config = update_config(config, args)

        from rl_games.torch_runner import Runner

        from aerial_gym.rl_training.rl_games.early_stop_a2c_agent import EarlyStopA2CAgent

        runner = Runner()
        # Reward-stability early stop — see YAML `early_stop_stable`.
        runner.algo_factory.register_builder(
            "a2c_continuous", lambda **kwargs: EarlyStopA2CAgent(**kwargs)
        )

        # Custom networks (select via params.network.name in the YAML).
        from rl_games.algos_torch import model_builder

        from aerial_gym.rl_training.rl_games.navrl_network import NavRLCNNBuilder
        from aerial_gym.rl_training.rl_games.navrl_vision_network import NavRLVisionBuilder
        from aerial_gym.rl_training.rl_games.navrl_vision_legacy_network import (
            NavRLVisionLegacyBuilder,
        )
        from aerial_gym.rl_training.rl_games.navrl_transformer_network import (
            NavRLTransformerBuilder,
        )

        model_builder.register_network("navrl_cnn", NavRLCNNBuilder)
        model_builder.register_network("navrl_vision", NavRLVisionBuilder)
        model_builder.register_network("navrl_vision_legacy", NavRLVisionLegacyBuilder)
        model_builder.register_network("navrl_transformer", NavRLTransformerBuilder)

        # Task-aware dashboard config: banner title, episode-length cap and the optional
        # reward "design range" hint all follow the task being trained.
        try:
            _dash_task = str(args.get("task") or config["params"]["config"].get("env_name", ""))
            from aerial_gym.rl_training.rl_games.run_header import _TASK_TITLES

            # Surface the run folder name (ppo_YYMMDD_HHMM_<task>) so the dashboard box title AND
            # the recurring "NavRL progress" line show which run this terminal is — avoids mixing up
            # concurrent runs. AERIAL_RUN_NAME is read by navrl_task._log_progress.
            _run_name = str(config["params"]["config"].get("full_experiment_name") or "").strip()
            os.environ.setdefault("AERIAL_RUN_NAME", _run_name)
            os.environ.setdefault(
                "AERIAL_RUN_TITLE",
                f"Aerial RL  ·  {_TASK_TITLES.get(_dash_task, _dash_task or 'PPO')}",
            )
            os.environ.setdefault(
                "AERIAL_TASK_CONFIG_MODULE",
                f"aerial_gym.config.task_config.{_dash_task}_config",
            )
            if _dash_task in ("position_setpoint_task", "shooting_moving_target_task"):
                os.environ.setdefault("AERIAL_REWARD_DESIGN_MAX", "1000")
        except Exception:
            pass

        # Prettier boxed stats only during TRAIN; PLAY 에서는 학습처럼 보이게 하지 않음.
        if not args.get("play"):
            try:
                _rl_here = Path(__file__).resolve().parent
                if str(_rl_here) not in sys.path:
                    sys.path.insert(0, str(_rl_here))
                from pretty_train_stats import install_boxed_statistics

                install_boxed_statistics()
            except Exception:
                pass

        try:
            if quiet_startup_enabled() and not args.get("play"):
                with devnull_stdio():
                    runner.load(config)
            else:
                runner.load(config)
        except yaml.YAMLError as exc:
            _real_print(exc)

        from aerial_gym.rl_training.rl_games.run_header import print_run_header

        print_run_header(
            args,
            config,
            mode="play" if args.get("play") else "train",
        )

    rank = int(os.getenv("LOCAL_RANK", "0"))
    if args["track"] and rank == 0:
        import wandb

        wandb.init(
            project=args["wandb_project_name"],
            entity=args["wandb_entity"],
            sync_tensorboard=True,
            config=config,
            monitor_gym=True,
            save_code=True,
        )
    if not args.get("play"):
        patch_rlgames_quiet_train_init()
    runner.run(args)

    if args["track"] and rank == 0:
        wandb.finish()
