# CLAUDE.md

Claude Code loads this file automatically. The canonical instructions for every coding and research
agent live in [`AGENTS.md`](AGENTS.md) and are imported here, so there is one source of rules:

@AGENTS.md

- 커밋/푸시 전 사용자가 diff 검토·승인한다. 작업 브랜치는 `main` 하나다 (단기 리뷰 브랜치·워크트리는 병합 후 삭제).
  Commit or push only after the user has reviewed and approved the diff; `main` is the working branch.
  `tests/test_branch_policy.py` reads this line together with root `OPERATIONS.md` §0.
- The observation contract that source comments call the "CLAUDE.md observation contract" is in
  `AGENTS.md` under *Research rules*.
- The previous contents of this file, including status notes through 2026-09-11, are in git history:
  `git show 7fb547d:CLAUDE.md`.
