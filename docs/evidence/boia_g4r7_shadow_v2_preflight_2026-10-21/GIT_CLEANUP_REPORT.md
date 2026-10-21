# G4-R7 Git cleanup report — research worktree display, main protection

2026-10-21 KST. Scope approved by the user for G4-R7: make the research worktree's Source
Control show nothing to stage, without hiding or harming protected assets, and re-check main.
No `git add .`, reset, clean, stash, gc, rebase, push or merge was used; nothing was deleted.

## Research worktree `src/MOTAR-mt-p1` (`codex/mt-p1`)

| | staged | unstaged | untracked shown |
|---|---|---|---|
| Before G4-R7 (HEAD `6bd986e`) | 0 | 0 | 2 (the protected symlinks) |
| After the source and evidence commits | 0 | 0 | 0 |

The two untracked symlinks are kept exactly as they were (same inode, mtime and target):

- `results/mt_p1_precision_v1` → `…/aerial_gym_simulator/results/mt_p1_precision_v1`
- `results/occlusion_pilot_v1` → `…/aerial_gym_simulator/results/occlusion_pilot_v1`

They are now excluded from display **in this worktree only**:

1. `git config extensions.worktreeConfig true` — one line added to the shared `.git/config`.
   No `config.worktree` existed anywhere before, and the shared config has no `core.worktree`
   and `core.bare = false`, so no other worktree's effective settings change apart from this
   flag. Git 2.25.1 supports the extension with `repositoryformatversion = 0`.
2. `git config --worktree core.excludesFile <file>` — written to
   `.git/worktrees/MOTAR-mt-p1/config.worktree`.
3. `<file>` = `.git/worktrees/MOTAR-mt-p1/motar_display_exclude`, outside every working tree,
   with exactly two anchored patterns: `/results/mt_p1_precision_v1` and
   `/results/occlusion_pilot_v1`. There was no global `core.excludesFile` and no
   `~/.config/git/ignore`, so this setting overrides nothing.

The shared `.git/info/exclude` was **not** changed (SHA-256 `b1373c93…` before and after).
`git check-ignore` matches only those two paths; any other new path under `results/` still
appears. The preflight guard for untracked runtime files covers `aerial_gym/ motar_eval/
tools/ tests/` and is unaffected.

## Checks after the change

| Check | Result |
|---|---|
| `git config --list` in all 9 worktrees | only `extensions.worktreeconfig=true` added; `core.excludesfile` only in MOTAR-mt-p1 |
| Status of the 8 other worktrees (HEAD, staged, unstaged, untracked set hash) | identical to before |
| Main `results/` stat fingerprint (path, size, mtime) | 41,939 files, 13,471,285,828 bytes, SHA-256 `31de0c32…` before and after |
| Main WIP (`.gitignore`, `WORKLOG.md`, `presentations/README.md`) | diff hash unchanged; not staged, stashed or reset |
| Stash `23ddb33` (user WIP, 2026-10-15) | present |

Backups taken before the change, outside any repository:
`/home/fair/workspaces/aerial_gym_ws/git_config_backups/2026-10-21_g4r7/`
`shared_config.backup` (`9eb7f5ac…`) and `shared_info_exclude.backup` (`b1373c93…`).

Undo, if wanted:

```bash
git -C /home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1 config --worktree --unset core.excludesFile
rm /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/.git/worktrees/MOTAR-mt-p1/motar_display_exclude
# only if no worktree uses config.worktree any more:
git -C /home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1 config --unset extensions.worktreeConfig
```

## Main checkout `src/aerial_gym_simulator` (`main`, HEAD `f74390a`) — unchanged

3 modified tracked files (user WIP) and 3,553 untracked: 3,474 under `results/` (historical raw,
the repository's "ignored but bound" convention), 78 under `presentations/professor_2026-10-01/`
and `tests/test_professor_presentation.py` (user material). No ignore policy was changed:
blanket-ignoring the raw would also hide future new raw. The user's WIP `.gitignore` edit turns
`*.pyc` into `c*.pyc`, which looks accidental (most `.pyc` would stop being ignored; 3 under
`results/**/__pycache__/` are visible now); it is the user's decision and was not touched.

## Git clean versus the IDE

`git status` in the research worktree is clean. VS Code, however, was last opened on the parent
folders `aerial_gym_ws` and `aerial_gym_ws/src`; from `src` its Git extension finds all three
sibling repositories, so main's 3,553 entries appear next to the research repository. That is
a window choice, not a Git state. Prepared (not verified in the GUI):
`/home/fair/workspaces/aerial_gym_ws/MOTAR-mt-p1-research.code-workspace`, a one-folder
workspace for `src/MOTAR-mt-p1` with `git.autoRepositoryDetection: "subFolders"` (files opened
through the two symlinks do not pull main into the view) and
`git.openRepositoryInParentFolders: "never"`. User action: **File → Open Workspace from File…**
→ that file, in a new window; open `src/aerial_gym_simulator` in a separate window for main work.

## Disk note

Free space fell from 8.77 GB to 7.97 GB during the session. The cause is `~/.cache/uv`
(785 MB written 13:46–14:07 KST by the claude-mem plugin installing its search server), not this
work; the shared repository's packs did not grow (a pack's mtime was only refreshed). Nothing was
removed. Free space stays above the 3.3 GB Q01-V2 preflight floor.
