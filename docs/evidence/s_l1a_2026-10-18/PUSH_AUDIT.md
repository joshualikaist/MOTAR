# Pre-push history audit for `codex/mt-p1` — 2026-10-18

This audit is the condition for the standing conditional authorization to push the research branch
`codex/mt-p1` normally. It does not cover a main merge, a main push or a force-push.

## Scope

Audited every commit reachable from HEAD and not from any remote-tracking ref (`git rev-list HEAD --not --remotes`):

- At `1e26d08` that is 48 commits: 783 new objects, of which 453 are blobs.
- The range includes local `main` commits up to `cb644bb` that were never pushed.
- Remote: `origin` = `github.com:joshualikaist/MOTAR`. `refs/heads/main` was `2da5b12`, matching
  the local tracking ref, and `codex/mt-p1` was absent from the remote.

[`push_audit/push_audit.py`](push_audit/push_audit.py) is read-only. Its output at `1e26d08` is
[`push_audit/push_audit_1e26d08.json`](push_audit/push_audit_1e26d08.json). The same script is
re-run on the final HEAD immediately before pushing.

## Results at `1e26d08`

| Check | Result |
| --- | --- |
| Author and committer | All 48 commits are `joshualikaist <kaist.mesc@gmail.com>`, the account owner |
| Checkpoints (`.pth/.pt/.ckpt/.onnx/.safetensors`) | 0 |
| Raw cells (`.npz/.npy/.bin`, frames, logs under raw roots) | 0 |
| Raw-like matches | Compact records only: 16 `results/kroc2027_*` JSON manifests, analyses and reports (≤ 50 KB each), tracked by design since `f9095c3`; one 175 KB `eq1b_scene_episodes.csv.gz` evidence table |
| Blobs over 2 MB | One: OCC-P2 `analysis.json`, 3.7 MB of text |
| Protected untracked `results/mt_p1_precision_v1`, `results/occlusion_pilot_v1` | Absent from every pushed object |
| Secret patterns (private keys, GitHub/AWS/Slack/OpenAI/Google tokens, quoted key/password assignments) | 0 hits in added lines |
| Sensitive filenames (`.env`, `id_rsa`, `*.pem`, `*.key`, credentials, `.netrc`) | 0 |
| Symlinks and submodules | 0 |
| Main-checkout user WIP (`.gitignore`, `WORKLOG.md`, `presentations/README.md`) | The current WIP blobs are absent from the push |
| Stash `23ddb33` | Not an ancestor of HEAD |

Local paths appear in 105 files, as `/home/fair` (506 tokens) and as `/tmp` scratch or test
directories. They occur in provenance records (run reports, manifests, preservation files) and in
test logs.

The published `origin/main` already contains the same `/home/fair/workspaces/aerial_gym_ws`
prefix in 1,899 files, and it already contains agent scratch paths such as `/tmp/claude-1000/…`.
The unpublished range therefore adds no new kind of host, user or credential information. The
paths are kept as provenance; history is not rewritten.

**Verdict: PASS.** A normal push of `codex/mt-p1` only is authorized, with no `main` ref and no
force. Any blocking finding on the final re-run stops the push.
