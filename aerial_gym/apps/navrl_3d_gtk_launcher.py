#!/usr/bin/python3
"""GTK launcher for the NavRL 3-D simulator.

The Isaac Gym environment runs in the aerialgym Conda interpreter.  This small launcher uses the
desktop's native GTK/Pango renderer, then starts that interpreter with ordinary CLI arguments.
"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk  # noqa: E402


HERE = Path(__file__).resolve()
REPO = HERE.parents[2]
APP = REPO / "aerial_gym" / "apps" / "navrl_3d.py"
RL_RUNS = REPO / "aerial_gym" / "rl_training" / "rl_games" / "runs"


CSS = b"""
window { background: #f3f6fb; }
.header { background: #14213d; padding: 28px 34px; }
.title { color: #ffffff; font-family: Arial, sans-serif; font-size: 30px; font-weight: 700; }
.subtitle { color: #c8d8f5; font-family: Arial, sans-serif; font-size: 15px; }
.card { background: #ffffff; border: 1px solid #d6deea; border-radius: 8px; padding: 22px; }
.card-title { color: #1c2b42; font-family: Arial, sans-serif; font-size: 18px; font-weight: 700; }
.section-title { color: #1e3a5f; font-family: Arial, sans-serif; font-size: 20px; font-weight: 700; }
.body { color: #52647d; font-family: Arial, sans-serif; font-size: 15px; }
.field-label { color: #26364f; font-family: Arial, sans-serif; font-size: 15px; font-weight: 600; }
.hint { color: #6c7b91; font-family: Arial, sans-serif; font-size: 13px; }
.status { background: #f5f7fa; border: 1px solid #d9e1ec; border-radius: 4px; padding: 12px; }
.status-ok { color: #15803d; font-family: Arial, sans-serif; font-size: 13px; font-weight: 600; }
.status-warn { color: #a95608; font-family: Arial, sans-serif; font-size: 13px; font-weight: 600; }
.status-error { color: #c62828; font-family: Arial, sans-serif; font-size: 13px; font-weight: 600; }
.controls { color: #31445f; font-family: Arial, sans-serif; font-size: 14px; font-weight: 600; }
button { font-family: Arial, sans-serif; font-size: 15px; padding: 10px 18px; }
.primary { background: #2563eb; color: #ffffff; border-color: #2563eb; font-weight: 700; }
.primary:hover { background: #1d4ed8; }
.secondary { background: #e8eef8; color: #20304a; border-color: #d7e0ee; font-weight: 700; }
spinbutton, filechooserbutton { font-family: Arial, sans-serif; font-size: 15px; }
"""


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-python", required=True)
    return parser.parse_args()


def inspect_checkpoint(runtime_python, path):
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
        [runtime_python, "-c", probe, str(path)],
        text=True,
        capture_output=True,
        check=False,
        env=dict(os.environ, PYTHONNOUSERSITE="1"),
    )
    try:
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError):
        return {"ok": False, "error": result.stderr.strip() or "Checkpoint inspection failed"}


def label(text, css_class, xalign=0.0):
    widget = Gtk.Label(label=text, xalign=xalign)
    widget.get_style_context().add_class(css_class)
    return widget


def card(title):
    outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=16)
    outer.get_style_context().add_class("card")
    outer.pack_start(label(title, "card-title"), False, False, 0)
    return outer


class Launcher:
    def __init__(self, runtime_python):
        self.runtime_python = str(Path(runtime_python).expanduser().resolve())
        self.mode = None
        self.window = Gtk.Window(title="NavRL 3D Simulator")
        self.window.set_default_size(1040, 700)
        self.window.set_size_request(920, 650)
        self.window.set_position(Gtk.WindowPosition.CENTER)
        self.window.connect("destroy", Gtk.main_quit)

        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.window.add(root)

        header = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        header.get_style_context().add_class("header")
        header.pack_start(label("NavRL 3D Simulator", "title"), False, False, 0)
        header.pack_start(
            label("Evaluate policy generalization in the real Isaac Gym environment", "subtitle"),
            False,
            False,
            0,
        )
        root.pack_start(header, False, False, 0)

        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=18)
        content.set_border_width(26)
        root.pack_start(content, True, True, 0)

        columns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=18)
        content.pack_start(columns, True, True, 0)
        columns.pack_start(self._evaluation_card(), True, True, 0)
        columns.pack_start(self._policy_card(), True, True, 0)
        content.pack_start(self._controls_card(), False, False, 0)

        footer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        manual = Gtk.Button(label="Open manual preview")
        manual.get_style_context().add_class("secondary")
        manual.connect("clicked", self._finish, "manual")
        policy = Gtk.Button(label="Start policy evaluation  >")
        policy.get_style_context().add_class("primary")
        policy.connect("clicked", self._finish, "policy")
        footer.pack_start(manual, False, False, 0)
        footer.pack_end(policy, False, False, 0)
        content.pack_end(footer, False, False, 0)

    def _evaluation_card(self):
        box = card("Evaluation setup")
        box.pack_start(label("Generalized evaluation", "section-title"), False, False, 0)
        description = label(
            "Runs 10 independent trials.\n"
            "Each trial randomizes 25-110 obstacles, drone position/yaw,\n"
            "and target position. The target moves throughout the trial.",
            "body",
        )
        description.set_line_wrap(True)
        box.pack_start(description, False, False, 0)

        grid = Gtk.Grid(column_spacing=18, row_spacing=8)
        grid.set_column_homogeneous(False)
        box.pack_start(grid, False, False, 8)

        grid.attach(label("Target speed", "field-label"), 0, 0, 1, 1)
        self.target_speed = Gtk.SpinButton.new_with_range(0.0, 2.5, 0.25)
        self.target_speed.set_value(0.75)
        self.target_speed.set_digits(2)
        grid.attach(self.target_speed, 1, 0, 1, 1)
        grid.attach(label("m/s | must be above zero in policy mode", "hint"), 0, 1, 2, 1)

        grid.attach(label("Drone max speed", "field-label"), 0, 2, 1, 1)
        self.drone_speed = Gtk.SpinButton.new_with_range(0.25, 3.0, 0.25)
        self.drone_speed.set_value(2.0)
        self.drone_speed.set_digits(2)
        grid.attach(self.drone_speed, 1, 2, 1, 1)
        return box

    def _policy_card(self):
        box = card("Policy model")
        box.pack_start(label("Trained checkpoint", "field-label"), False, False, 0)
        self.checkpoint = Gtk.FileChooserButton.new(
            "Select a PyTorch checkpoint", Gtk.FileChooserAction.OPEN
        )
        if RL_RUNS.is_dir():
            self.checkpoint.set_current_folder(str(RL_RUNS))
        file_filter = Gtk.FileFilter()
        file_filter.set_name("PyTorch checkpoint (*.pth)")
        file_filter.add_pattern("*.pth")
        self.checkpoint.add_filter(file_filter)
        self.checkpoint.connect("file-set", self._checkpoint_changed)
        box.pack_start(self.checkpoint, False, False, 0)

        self.status = label("Select a checkpoint to inspect model compatibility.", "hint")
        self.status.set_line_wrap(True)
        self.status.get_style_context().add_class("status")
        box.pack_start(self.status, False, False, 0)

        box.pack_start(label("Model compatibility", "hint"), False, False, 0)
        details = label(
            "574D Transformer: RGB-D/LiDAR perception\n"
            "305D legacy CNN: archived semantic baseline\n"
            "The matching runtime is selected automatically.",
            "body",
        )
        details.set_line_wrap(True)
        box.pack_start(details, False, False, 0)
        return box

    def _controls_card(self):
        box = card("Viewer controls")
        controls = label(
            "Target speed  , / .       Drone speed  - / =       LiDAR  G       New trial  N\n"
            "Policy / Manual  M       Move  I K J L       Yaw  U O       Pause  Space",
            "controls",
            xalign=0.5,
        )
        box.pack_start(controls, False, False, 0)
        return box

    def _set_status(self, text, css_class):
        context = self.status.get_style_context()
        for old in ("status-ok", "status-warn", "status-error", "hint"):
            context.remove_class(old)
        context.add_class(css_class)
        self.status.set_text(text)

    def _checkpoint_changed(self, _widget):
        filename = self.checkpoint.get_filename()
        if not filename:
            self._set_status("No checkpoint selected.", "hint")
            return None
        info = inspect_checkpoint(self.runtime_python, filename)
        if not info.get("ok"):
            self._set_status("Read failed: %s" % info.get("error", "unknown"), "status-error")
            return None
        labels = {
            "transformer": "NavRL++ Target Transformer | 574D | Recommended",
            "legacy_vision_305": "Legacy semantic Vision CNN | 305D | Baseline playback",
            "vision_1265": "RGB-D + semantic LiDAR Vision CNN | 1265D",
        }
        text = labels.get(info.get("kind"))
        if text is None:
            self._set_status("Unsupported checkpoint architecture.", "status-error")
            return None
        if info.get("epoch") is not None:
            text += " | epoch %s" % info["epoch"]
        self._set_status(text, "status-ok" if info["kind"] == "transformer" else "status-warn")
        return info

    def _error(self, title, message):
        dialog = Gtk.MessageDialog(
            transient_for=self.window,
            flags=Gtk.DialogFlags.MODAL,
            message_type=Gtk.MessageType.ERROR,
            buttons=Gtk.ButtonsType.OK,
            text=title,
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()

    def _finish(self, _button, mode):
        if mode == "policy":
            filename = self.checkpoint.get_filename()
            if not filename:
                self._error("Checkpoint required", "Select a trained .pth checkpoint.")
                return
            if self.target_speed.get_value() <= 0.0:
                self._error("Invalid target speed", "Policy evaluation requires a moving target.")
                return
            if self._checkpoint_changed(self.checkpoint) is None:
                self._error("Incompatible checkpoint", self.status.get_text())
                return
        self.mode = mode
        self.window.destroy()

    def run(self):
        self.window.show_all()
        Gtk.main()
        if self.mode is None:
            return 0
        command = [
            self.runtime_python,
            str(APP),
            "--target-speed",
            str(self.target_speed.get_value()),
            "--drone-speed",
            str(self.drone_speed.get_value()),
        ]
        if self.mode == "manual":
            command.append("--manual")
        else:
            command.extend(("--checkpoint", self.checkpoint.get_filename()))
        return subprocess.call(command, cwd=str(REPO), env=dict(os.environ, PYTHONNOUSERSITE="1"))


def main():
    args = parse_args()
    provider = Gtk.CssProvider()
    provider.load_from_data(CSS)
    Gtk.StyleContext.add_provider_for_screen(
        Gdk.Screen.get_default(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
    )
    return Launcher(args.runtime_python).run()


if __name__ == "__main__":
    raise SystemExit(main())
