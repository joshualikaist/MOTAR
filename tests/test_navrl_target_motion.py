import importlib.util
from pathlib import Path

import torch


_MODULE_PATH = (
    Path(__file__).parents[1]
    / "aerial_gym"
    / "task"
    / "navrl_task"
    / "target_motion.py"
)
_SPEC = importlib.util.spec_from_file_location("navrl_target_motion_standalone", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
steer_target_step = _MODULE.steer_target_step


def _bounds(n):
    lo = torch.full((n, 2), -10.0)
    hi = torch.full((n, 2), 10.0)
    return lo, hi


def test_unobstructed_step_preserves_heading_and_speed():
    old = torch.tensor([[0.0, 0.0]])
    desired = torch.tensor([[1.2, 1.6]])
    speed = torch.tensor([2.0])
    lo, hi = _bounds(1)
    new, velocity, steered, clear = steer_target_step(
        old, desired, speed, 0.1, torch.empty(1, 0, 2), lo, hi, 1.0, torch.ones(1)
    )
    assert not bool(steered.item())
    assert bool(clear.item())
    assert torch.allclose(velocity, desired)
    assert torch.allclose((new - old).norm(dim=1), torch.tensor([0.2]))


def test_bar_ahead_selects_a_full_speed_side_step():
    old = torch.tensor([[0.0, 0.0]])
    desired = torch.tensor([[1.0, 0.0]])
    speed = torch.tensor([1.0])
    # Direct 0.1 m motion violates the 1 m disc; +/-90 candidates are clear.
    bars = torch.tensor([[[1.05, 0.0]]])
    lo, hi = _bounds(1)
    new, velocity, steered, clear = steer_target_step(
        old, desired, speed, 0.1, bars, lo, hi, 1.0, torch.ones(1)
    )
    assert bool(steered.item())
    assert bool(clear.item())
    assert torch.allclose((new - old).norm(dim=1), torch.tensor([0.1]), atol=1e-6)
    assert torch.allclose(velocity.norm(dim=1), speed)
    assert torch.cdist(new.unsqueeze(1), bars).item() >= 1.0


def test_tie_break_is_mirror_symmetric():
    old = torch.zeros(2, 2)
    desired = torch.tensor([[1.0, 0.0], [1.0, 0.0]])
    speed = torch.ones(2)
    bars = torch.tensor([[[1.05, 0.0]], [[1.05, 0.0]]])
    lo, hi = _bounds(2)
    new, velocity, steered, clear = steer_target_step(
        old,
        desired,
        speed,
        0.1,
        bars,
        lo,
        hi,
        1.0,
        torch.tensor([1.0, -1.0]),
    )
    assert bool(steered.all())
    assert bool(clear.all())
    assert torch.allclose(new[0, 0], new[1, 0])
    assert torch.allclose(new[0, 1], -new[1, 1])
    assert torch.allclose(velocity[0, 1], -velocity[1, 1])


def test_heading_continuity_turns_toward_previous_flight_direction():
    old = torch.tensor([[0.0, 0.0]])
    desired = torch.tensor([[1.0, 0.0]])
    speed = torch.tensor([1.0])
    lo, hi = _bounds(1)
    new, velocity, steered, clear = steer_target_step(
        old,
        desired,
        speed,
        0.1,
        torch.empty(1, 0, 2),
        lo,
        hi,
        1.0,
        torch.ones(1),
        torch.tensor([torch.pi]),
    )
    heading = torch.atan2(velocity[:, 1], velocity[:, 0]).abs()
    assert bool(steered.item())
    assert bool(clear.item())
    assert torch.allclose(heading, torch.tensor([torch.pi / 2]), atol=1e-6)
    assert torch.allclose((new - old).norm(dim=1), torch.tensor([0.1]), atol=1e-6)


def test_heading_window_is_preference_not_escape_veto():
    old = torch.tensor([[0.0, 0.0]])
    desired = torch.tensor([[-1.0, 0.0]])
    speed = torch.tensor([1.0])
    lo, hi = _bounds(1)
    angles = torch.tensor(
        [torch.deg2rad(torch.tensor(value)) for value in _MODULE.TURN_ANGLES_DEG]
    )
    base = desired[0]
    directions = torch.stack(
        (
            base[0] * torch.cos(angles) - base[1] * torch.sin(angles),
            base[0] * torch.sin(angles) + base[1] * torch.cos(angles),
        ),
        dim=1,
    )
    # Block every candidate except the direct west escape. Its heading differs from the previous
    # eastward flight by 180 degrees, so selecting it proves that the 90-degree window is no veto.
    bars = (directions[1:] * 0.1).unsqueeze(0)
    new, velocity, steered, clear = steer_target_step(
        old,
        desired,
        speed,
        0.1,
        bars,
        lo,
        hi,
        0.02,
        torch.ones(1),
        torch.tensor([0.0]),
    )
    assert not bool(steered.item())
    assert bool(clear.item())
    assert torch.allclose(velocity, desired)
    assert torch.allclose(new, torch.tensor([[-0.1, 0.0]]), atol=1e-6)
