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


class ExtractObsWrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)

    def reset(self, **kwargs):
        observations, *_ = super().reset(**kwargs)
        return observations["observations"]

    def step(self, action):
        observations, rewards, terminated, truncated, infos = super().step(action)

        dones = torch.where(
            terminated | truncated,
            torch.ones_like(terminated),
            torch.zeros_like(terminated),
        )

        return (
            observations["observations"],
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

    def reset_done(self):
        return self.env.reset_done()

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


def _run_name_from_checkpoint(ckpt_path, base_dir=None):
    """Extract runs/<run_name>/ from .../runs/<run_name>/nn/<file>.pth."""
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
        return run_root.name
    return None


def _task_run_suffix(task: Optional[str]) -> str:
    """Short label appended to run folder names (fixed vs moving intercept, etc.)."""
    if not task:
        return ""
    mapping = {
        "position_setpoint_task": "fixed",
        "shooting_moving_target_task": "moving",
        "navigation_task": "nav",
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


def update_config(config, args):

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
            _gn = 10
        player_cfg["games_num"] = max(1, _gn)
    config["params"]["config"]["player"] = player_cfg

    # Runs folder: ppo_YYMMDD_HHMM_<fixed|moving|…> (local time).
    # Resume/play with --checkpoint: reuse runs/<name>/ from the checkpoint path so TensorBoard,
    # nn/, and aerial_run/ stay in one folder instead of splitting per launch.
    _cfg = config["params"]["config"]
    fe = _cfg.get("full_experiment_name", None)
    if isinstance(fe, str) and fe.strip():
        pass
    else:
        resumed = _run_name_from_checkpoint(args.get("checkpoint"))
        if resumed:
            _cfg["full_experiment_name"] = resumed
        else:
            task = args.get("task") or _cfg.get("env_name")
            _cfg["full_experiment_name"] = _new_experiment_name(task)

    return config


if __name__ == "__main__":
    os.makedirs("nn", exist_ok=True)
    os.makedirs("runs", exist_ok=True)

    install_quiet_print_filter()
    mute_info_logging()

    args = vars(get_args())
    args = _prepare_rlgames_argv_dict(args)

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
