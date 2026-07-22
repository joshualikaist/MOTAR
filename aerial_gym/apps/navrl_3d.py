#!/usr/bin/env python3
"""Native interactive NavRL 3-D application.

This is not a Three.js mock-up: it runs the real Isaac Gym task, Warp LiDAR, RGB-D perception,
and (when selected) an rl_games Transformer checkpoint. With no checkpoint it opens a manual
sensor/environment demo so the application can be inspected before policy training finishes.
"""

import argparse
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys


HERE = Path(__file__).resolve()
REPO = HERE.parents[2]
RL_DIR = REPO / "aerial_gym" / "rl_training" / "rl_games"


def _args():
    parser = argparse.ArgumentParser(description="NavRL interactive Isaac Gym application")
    parser.add_argument("--checkpoint", type=Path, help="supported NavRL .pth checkpoint")
    parser.add_argument("--manual", action="store_true", help="run without a policy checkpoint")
    parser.add_argument("--bars", type=int, default=48, help=argparse.SUPPRESS)
    parser.add_argument("--target-speed", type=float, default=0.75)
    parser.add_argument("--drone-speed", type=float, default=2.0)
    parser.add_argument("--num-envs", type=int, default=1)
    return parser.parse_args()


def _inspect_checkpoint(path):
    """Inspect checkpoint ABI in a subprocess so torch is not imported before Isaac Gym here."""
    probe = r"""
import json, sys, torch
try:
    ck = torch.load(sys.argv[1], map_location='cpu', weights_only=False)
    model = ck.get('model', {})
    keys = list(model.keys())
    dim = None
    for key, value in model.items():
        if key.endswith('running_mean_std.running_mean') and getattr(value, 'ndim', 0) == 1:
            dim = int(value.shape[0])
            if dim > 1:
                break
    if any('.cls_token' in key for key in keys):
        kind = 'transformer'
    elif any('.scan_cnn.' in key for key in keys):
        kind = 'legacy_vision_305'
    elif any('.lidar_cnn.' in key for key in keys):
        kind = 'vision_1265'
    else:
        kind = 'unsupported'
    print(json.dumps({'ok': True, 'kind': kind, 'obs_dim': dim, 'epoch': ck.get('epoch')}))
except Exception as exc:
    print(json.dumps({'ok': False, 'error': str(exc)}))
"""
    result = subprocess.run(
        [sys.executable, "-c", probe, str(path)],
        text=True,
        capture_output=True,
        check=False,
    )
    try:
        info = json.loads(result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError):
        info = {"ok": False, "error": result.stderr.strip() or "checkpoint inspection failed"}
    return info


def _configure_checkpoint(args):
    path = Path(args.checkpoint).expanduser().resolve()
    if not path.is_file():
        raise ValueError("Checkpoint file not found.")
    info = _inspect_checkpoint(path)
    if not info.get("ok"):
        raise ValueError("Unable to read checkpoint: %s" % info.get("error", "unknown"))
    kind = info.get("kind")
    expected = {"transformer": 574, "legacy_vision_305": 305, "vision_1265": 1265}
    if kind not in expected:
        raise ValueError("Unsupported checkpoint architecture. Check its CNN/LSTM type.")
    observed = info.get("obs_dim")
    if observed not in (None, expected[kind]):
        raise ValueError(
            "Checkpoint observation is %sD, but the detected model expects %sD."
            % (observed, expected[kind])
        )
    args.checkpoint = path
    args.policy_kind = kind
    args.checkpoint_info = info
    return info


def _setup_dialog(args):
    """Large high-DPI launcher; live controls continue inside the 3-D window."""
    if args.manual or args.checkpoint is not None:
        return args
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
    except ImportError as exc:
        raise SystemExit("tkinter is unavailable; pass --manual or --checkpoint PATH") from exc

    root = tk.Tk()
    root.title("NavRL 3D Simulator")
    root.geometry("1040x730")
    root.minsize(920, 680)
    root.configure(bg="#F4F7FB")
    root.tk.call("tk", "scaling", 1.45)

    from tkinter import font as tkfont

    # This Tk build maps explicit Linux font-family names to a jagged bitmap renderer. Clone its
    # native UI font instead: buttons/dialogs already render this font correctly, and the copies
    # let us keep presentation-sized typography without overriding the system font family.
    base_font = tkfont.nametofont("TkDefaultFont")

    def sized_font(size, bold=False):
        value = base_font.copy()
        value.configure(size=size, weight="bold" if bold else "normal")
        return value

    title_font = sized_font(27, bold=True)
    section_font = sized_font(15, bold=True)
    card_title_font = sized_font(14, bold=True)
    body_font = sized_font(12)
    copy_font = sized_font(11)
    hint_font = sized_font(10)
    hint_bold_font = sized_font(10, bold=True)
    button_font = sized_font(11, bold=True)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame", background="#F4F7FB")
    style.configure("Card.TLabelframe", background="#FFFFFF", borderwidth=1, relief="solid")
    style.configure(
        "Card.TLabelframe.Label",
        background="#FFFFFF",
        foreground="#18253B",
        font=card_title_font,
    )
    style.configure("TLabel", background="#FFFFFF", foreground="#26364F", font=body_font)
    style.configure("Hint.TLabel", foreground="#66758C", font=hint_font)
    style.configure("TEntry", font=copy_font, padding=7)
    style.configure("TSpinbox", font=copy_font, padding=5)
    style.configure("TButton", font=button_font, padding=(14, 10))
    style.configure(
        "Accent.TButton", background="#2563EB", foreground="#FFFFFF", borderwidth=0
    )
    style.map("Accent.TButton", background=[("active", "#1D4ED8")])
    style.configure(
        "Manual.TButton", background="#E8EEF8", foreground="#20304A", borderwidth=0
    )
    checkpoint = tk.StringVar(value="")
    target_speed = tk.StringVar(value=str(args.target_speed))
    drone_speed = tk.StringVar(value=str(args.drone_speed))
    compatibility = tk.StringVar(value="Select a checkpoint to inspect model compatibility.")
    compatibility_color = {"value": "#66758C"}
    result = {"mode": None}

    def set_compatibility(text, color):
        compatibility.set(text)
        compatibility_color["value"] = color
        status_label.configure(fg=color)

    def inspect_selected(show_error=False):
        raw = checkpoint.get().strip()
        if not raw:
            set_compatibility("No checkpoint selected. Manual preview is available.", "#66758C")
            return None
        path = Path(raw).expanduser()
        if not path.is_file():
            set_compatibility("File not found.", "#DC2626")
            return None
        info = _inspect_checkpoint(path)
        if not info.get("ok"):
            set_compatibility("Read failed: %s" % info.get("error", "unknown"), "#DC2626")
            return None
        labels = {
            "transformer": "NavRL++ Target Transformer | 574D | Recommended",
            "legacy_vision_305": "Legacy semantic Vision CNN | 305D | Baseline playback",
            "vision_1265": "RGB-D + semantic LiDAR Vision CNN · 1265D",
        }
        label = labels.get(info.get("kind"))
        if label is None:
            set_compatibility("Unsupported checkpoint architecture.", "#DC2626")
            if show_error:
                messagebox.showerror("Incompatible checkpoint", compatibility.get())
            return None
        epoch = info.get("epoch")
        suffix = " | epoch %s" % epoch if epoch is not None else ""
        set_compatibility(label + suffix, "#15803D" if info["kind"] == "transformer" else "#B45309")
        return info

    def browse():
        value = filedialog.askopenfilename(
            title="Perception Transformer checkpoint",
            initialdir=str(RL_DIR / "runs"),
            filetypes=(("PyTorch checkpoint", "*.pth"), ("All files", "*")),
        )
        if value:
            checkpoint.set(value)
            inspect_selected()

    def finish(mode):
        try:
            args.target_speed = float(target_speed.get())
            args.drone_speed = float(drone_speed.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Target and drone speeds must be numbers.")
            return
        if mode == "policy" and args.target_speed <= 0.0:
            messagebox.showerror("Invalid input", "Policy evaluation requires target speed above zero.")
            return
        args.num_envs = 1
        if mode == "policy":
            path = Path(checkpoint.get()).expanduser()
            if not path.is_file():
                messagebox.showerror("Checkpoint required", "Select a trained .pth checkpoint.")
                return
            args.checkpoint = path
            try:
                _configure_checkpoint(args)
            except ValueError as exc:
                messagebox.showerror("Checkpoint compatibility error", str(exc))
                inspect_selected(show_error=False)
                return
        else:
            args.manual = True
        result["mode"] = mode
        root.destroy()

    header = tk.Frame(root, bg="#14213D", height=132)
    header.pack(fill="x")
    header.pack_propagate(False)
    tk.Label(
        header, text="NavRL 3D Simulator", bg="#14213D", fg="#FFFFFF",
        font=title_font,
    ).pack(anchor="w", padx=34, pady=(23, 0))
    tk.Label(
        header,
        text="Evaluate policy generalization in the real Isaac Gym environment",
        bg="#14213D", fg="#C5D7F8", font=copy_font,
    ).pack(anchor="w", padx=36, pady=(4, 18))

    main = ttk.Frame(root, padding=(26, 20, 26, 12))
    main.pack(fill="both", expand=True)
    main.columnconfigure(0, weight=1)
    main.columnconfigure(1, weight=1)

    scene = ttk.LabelFrame(main, text="  Evaluation setup  ", style="Card.TLabelframe", padding=18)
    scene.grid(row=0, column=0, sticky="nsew", padx=(0, 9))
    model = ttk.LabelFrame(main, text="  Policy model  ", style="Card.TLabelframe", padding=18)
    model.grid(row=0, column=1, sticky="nsew", padx=(9, 0))

    tk.Label(
        scene,
        text="Generalized evaluation",
        bg="#FFFFFF", fg="#1E3A5F", font=section_font,
    ).grid(row=0, column=0, columnspan=2, sticky="w")
    tk.Label(
        scene,
        text="Runs 10 independent trials.\n"
             "Each trial randomizes 25-110 obstacles,\n"
             "drone position/yaw, and target position.",
        bg="#FFFFFF", fg="#586A82", justify="left", font=copy_font,
    ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 22))

    ttk.Label(scene, text="Target speed").grid(row=2, column=0, sticky="w")
    ttk.Spinbox(
        scene, from_=0.0, to=2.5, increment=0.25, textvariable=target_speed, width=8
    ).grid(row=2, column=1, sticky="e")
    ttk.Label(scene, text="m/s | target always moves in policy mode", style="Hint.TLabel").grid(
        row=3, column=0, columnspan=2, sticky="w", pady=(4, 16)
    )
    ttk.Label(scene, text="Drone max speed").grid(row=4, column=0, sticky="w")
    ttk.Spinbox(
        scene, from_=0.25, to=3.0, increment=0.25, textvariable=drone_speed, width=8
    ).grid(row=4, column=1, sticky="e")

    ttk.Label(model, text="Trained checkpoint").pack(anchor="w")
    ck_row = ttk.Frame(model, style="TFrame")
    ck_row.pack(fill="x", pady=(7, 10))
    ttk.Entry(ck_row, textvariable=checkpoint).pack(side="left", fill="x", expand=True)
    ttk.Button(ck_row, text="Browse", command=browse).pack(side="left", padx=(8, 0))
    status_box = tk.Frame(model, bg="#F5F7FA", highlightbackground="#D9E1EC", highlightthickness=1)
    status_box.pack(fill="x", pady=(2, 16))
    status_label = tk.Label(
        status_box, textvariable=compatibility, bg="#F5F7FA", fg="#66758C",
        justify="left", anchor="w", wraplength=390, font=hint_bold_font,
    )
    status_label.pack(fill="x", padx=12, pady=11)
    ttk.Label(model, text="Model compatibility", style="Hint.TLabel").pack(anchor="w")
    tk.Label(
        model,
        text="574D Transformer: RGB-D/LiDAR perception\n"
             "305D legacy CNN: archived semantic baseline\n"
             "The launcher selects the matching runtime automatically.",
        bg="#FFFFFF", fg="#40516B", justify="left", anchor="w",
        font=hint_font,
    ).pack(fill="x", pady=(5, 0))

    controls = ttk.LabelFrame(main, text="  Viewer controls  ", style="Card.TLabelframe", padding=14)
    controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(18, 0))
    tk.Label(
        controls,
        text="Target speed  , / .     Drone speed  - / =     LiDAR  G     New trial  N\n"
             "Policy / Manual  M     Move  I K J L     Yaw  U O     Pause  Space",
        bg="#FFFFFF", fg="#31445F", justify="center", font=button_font,
    ).pack(fill="x", pady=2)

    footer = ttk.Frame(root, padding=(26, 4, 26, 22))
    footer.pack(fill="x")
    ttk.Button(
        footer, text="Open manual preview", style="Manual.TButton",
        command=lambda: finish("manual"),
    ).pack(side="left")
    ttk.Button(
        footer, text="Start policy evaluation  >", style="Accent.TButton",
        command=lambda: finish("policy"),
    ).pack(side="right")
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()
    if result["mode"] is None:
        raise SystemExit(0)
    return args


def _set_environment(args):
    os.environ["PYTHONNOUSERSITE"] = "1"
    # The native application always runs one environment. Do not reserve base_sim's PhysX contact
    # buffers sized for 8000 parallel environments; that can OOM when a training job is resident.
    os.environ["AERIAL_GYM_SIM_NAME"] = "navrl_viewer_sim"
    os.environ["NAVRL_INTERACTIVE"] = "1"
    os.environ["NAVRL_VISION"] = "1"
    os.environ["NAVRL_DENSITY_CURRICULUM"] = "0"
    os.environ.pop("NAVRL_NUM_BARS", None)
    os.environ["NAVRL_GENERAL_EVAL"] = "1"
    os.environ["NAVRL_GENERAL_DENSITY_MIN"] = "25"
    os.environ["NAVRL_GENERAL_DENSITY_MAX"] = "110"
    os.environ["NAVRL_TARGET_SPEED"] = str(max(0.0, args.target_speed))
    os.environ["NAVRL_TARGET_PATTERN"] = "mixed"
    os.environ["NAVRL_MAX_VELOCITY"] = str(max(0.25, args.drone_speed))
    kind = getattr(args, "policy_kind", "transformer")
    if kind == "legacy_vision_305":
        os.environ["NAVRL_PERCEPTION"] = "0"
        os.environ["NAVRL_LEGACY_VISION"] = "1"
        os.environ["NAVRL_NETWORK_OVERRIDE"] = "navrl_vision_legacy"
    elif kind == "vision_1265":
        os.environ["NAVRL_PERCEPTION"] = "0"
        os.environ["NAVRL_LEGACY_VISION"] = "0"
        os.environ.pop("NAVRL_NETWORK_OVERRIDE", None)
    else:
        os.environ["NAVRL_PERCEPTION"] = "1"
        os.environ["NAVRL_LEGACY_VISION"] = "0"
        os.environ.pop("NAVRL_NETWORK_OVERRIDE", None)


def _run_manual(args):
    # Isaac Gym must be imported before torch in this environment.
    import isaacgym  # noqa: F401
    import torch
    from aerial_gym.registry.task_registry import task_registry

    task = task_registry.make_task("navrl_task", headless=False, num_envs=1, use_warp=True)
    task.reset()
    task._interactive_manual = True
    actions = torch.zeros(
        (task.num_envs, task.task_config.action_space_dim), device=task.device
    )
    print("NavRL 3D manual mode: I/K/J/L move, U/O yaw, M toggles control.")
    with torch.no_grad():
        completed = 0
        while completed < 10:
            _, _, terminated, truncated, _ = task.step(actions)
            newly_finished = int(torch.sum(terminated | truncated).item())
            completed += newly_finished
            if newly_finished:
                print("NavRL 3D manual trial: %d/10" % min(completed, 10))
                if completed < 10:
                    task.reset()


def _run_policy(args):
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        raise SystemExit("checkpoint not found: %s" % checkpoint)
    os.environ["PLAY_GAMES_NUM"] = "10"
    config_file = (
        "ppo_navrl_perception_transformer.yaml"
        if args.policy_kind == "transformer"
        else "ppo_navrl_vision.yaml"
    )
    runner = RL_DIR / "runner.py"
    old_cwd = Path.cwd()
    try:
        os.chdir(RL_DIR)
        sys.argv = [
            str(runner),
            "--file", config_file,
            "--task", "navrl_task",
            "--num_envs", "1",
            "--headless", "False",
            "--use_warp", "True",
            "--play",
            "--checkpoint", str(checkpoint),
        ]
        runpy.run_path(str(runner), run_name="__main__")
    finally:
        os.chdir(old_cwd)


def main():
    args = _setup_dialog(_args())
    if not args.manual and not hasattr(args, "policy_kind"):
        try:
            _configure_checkpoint(args)
        except ValueError as exc:
            raise SystemExit("Checkpoint compatibility error: %s" % exc)
    _set_environment(args)
    if args.manual:
        _run_manual(args)
    else:
        _run_policy(args)


if __name__ == "__main__":
    main()
