"""07 §11 [NEW]: pure terminal-first state reset, independent of simulator outcomes."""
import torch

from .arbitration import BoiaState, Mode


def apply(terminal_prev, state):
    """Reset only current flagged rows; other rows are copied and must be reclassified."""
    assert terminal_prev.shape == state.mode.shape and terminal_prev.dtype == torch.bool
    terminal = terminal_prev
    return BoiaState(torch.where(terminal, torch.full_like(state.mode, Mode.TERMINAL), state.mode),
                     state.engaged.clone() & ~terminal,
                     torch.where(terminal, torch.zeros_like(state.dwell), state.dwell),
                     torch.where(terminal, torch.zeros_like(state.off_count), state.off_count),
                     torch.where(terminal, torch.zeros_like(state.miss_streak), state.miss_streak),
                     torch.where(terminal[:, None], torch.zeros_like(state.a_prev), state.a_prev),
                     state.ever_detected.clone() & ~terminal)
