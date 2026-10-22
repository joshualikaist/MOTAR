# MOTAR website and README progress update

Date: 22 October 2026 KST. Publishing candidate built on a fresh worktree at remote main
2da5b12c1954ee768fcba59d1fb1fb9c78f7d536. This isolates the BOIA documentation update from
20 later local-only main commits and keeps the user's local Main WIP untouched.

## Research snapshot

The site records Q01 and Q02-A from their reviewed source commits, and the latest Q02-B/Q03
review handoff at local research commit fde21cdf969a7df380aa1c4deddc1824cb524506.

- Q01-V2 passed bounded native/OFF/ON shadow parity; it does not establish algorithm efficacy.
- Q02-A v2.6 remains PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS, with residual P2 H1/H2.
- Q02-B and Q03 have focused CPU tests, but their Phase 6 independent review is FAIL as a stage gate:
  0 P0, 2 reproduced and unfixed P1, and 9 P2. Both packages remain HOLD.
- Q04-D0 is design-only and HOLD. No online intervention or executor exists.
- The six focused package suites report 112 tests OK. The latest PUBLIC_CPU receipt reports 3,177 run,
  zero failures, 10 skips and four known Warp errors, so the aggregate remains FAIL.
- Actual occlusion accuracy, persistent target identity, capture benefit, physical safety and real-flight
  performance are not established. G5 remains HOLD.

## Site change

The project and evidence pages add the BOIA extension and update milestone status to the latest review.
The five-step schematic remains an architecture explanation, not a rollout. A source map and machine
record bind claims to local commit IDs and SHA-256 receipts. The README adds the same scope and limits.
Existing scientific results, figures, viewer semantics, RGB-D + LiDAR contract, Frozen F checkpoint
and actor remain unchanged. No research code, Raw results or checkpoints are included.

## Validation

The browser comparison uses remote main 2da5b12 as its before state and the publishing candidate as
its after state. Desktop, tablet and mobile screenshots are regenerated in software-rendered Chrome.
The documentation tests, public-doc checker, local HTML links, page anchors, and existing site/viewer
JavaScript contracts are rerun against this candidate. See PREVIEW.md and the generated receipts.

## Publication

The only intended remote update is a fast-forward docs commit on main whose parent is the current
remote main. It excludes the 20 local-only main commits, the user's tracked and untracked WIP,
research implementation, Raw results, checkpoints and stash. Push and Pages verification status
are reported with the final Git traceability after the remote check.
