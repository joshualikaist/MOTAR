# Precision-lineage evidence review, 2026-10-16

This package contains small byte-identical copies of compact reports and manifests plus an independent CPU audit at source HEAD `c038ba5`. It preserves reviewable evidence without staging large raw episode files or following the untracked results symlink into Git. No source result, archive, checkpoint or preregistration was modified; no GPU, simulator, training, merge, push or commit was run for this audit.

## Storage and retention

Physical results live at `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/mt_p1_precision_v1`. The MT-P1 worktree path is a symlink to those same results, not a second raw copy. Archives actually live at `/home/fair/workspaces/aerial_gym_ws/kroc2027_raw_backup`; `../kroc2027_raw_backup` from the MT-P1 checkout is not that location.

Raw results and archives have the same filesystem device identifier. Classify these as **local same-filesystem copies, not external backups**. An independently verified external copy is still needed for disk-loss protection; none was created in this audit. Keep existing raw data, archives and the untracked symlink. Compact reports cannot replace the episode ledgers.

| Run | Manifest entries / SHA matches | Raw bytes | Archive bytes | Archive regular files | Manifest payload matches |
| --- | ---: | ---: | ---: | ---: | ---: |
| MT-P2-C01 | 131 / 131 | 49827428 | 6900241 | 132 | 131 |
| KROC27-A-M02 | 452 / 452 | 493152134 | 66951880 | 453 | 452 |

Every manifest entry was rehashed from the physical raw directory. Archive SHA-256 was checked against its existing sidecar; every regular tar member was streamed and hashed without extracting or copying the large payload. The archive contains exactly the manifest-listed files plus `RAW_MANIFEST.json`; the archived manifest matches the current manifest, with no unexpected files, path anomalies or links. Full hashes and paths are in `preservation_audit.json`; `index.json` binds every copied compact file to its original bytes.

Archive hashes:

- `mt_p2_c01_2026-10-16.tar.gz`: `2c2c3dd5005150a38cb4480e3f73af48177842da3ea6f3f860800d2d550d32f4`.
- `kroc27_a_m02_2026-10-16.tar.gz`: `3460335ff70612099a006b9c7c6ff19e5fb6b0dfce372d69e271c32196e44e79`.

## Independent numerical check

The 56 new A3/A4 cells were joined analytically with the 56 reused A1/A2 cells: 112 cells and 229,376 primary episodes. Every cell contributed exactly indices 0–15 from each of 128 environments. The original static manifest had 393 entries, all rehashed successfully and linked to amendment 2. Raw ledgers, rather than the published analysis means, supplied the counts.

| Arm | Capture count | Crash count | Timeout count | Capture % |
| --- | ---: | ---: | ---: | ---: |
| A1 | 56288 | 1056 | 0 | 98.158482 |
| A2 | 54659 | 2498 | 187 | 95.317732 |
| A3 | 54697 | 2638 | 9 | 95.383998 |
| A4 | 53218 | 4082 | 44 | 92.804827 |

Per seed the four densities were weighted equally. Independent paired-t (df 6), Holm over Delta_A/I_A, BCa 20,000 resamples with seed 4200, and all 128 exact sign-flip patterns reproduce the stored analysis. Details and seed effects are in `numerical_audit.json`.

| Contrast | Mean pp | 95% t CI pp | Holm p | Exact sign-flip p |
| --- | ---: | --- | ---: | ---: |
| Delta_A | +2.709961 | [+2.452724, +2.967198] | 4.49359198447e-07 | 0.015625 |
| I_A | -0.261579 | [-0.801079, +0.277920] | 0.280308941982 | 0.328125 |

These match the recorded `SUPPORTED` Delta_A and `INCONCLUSIVE` I_A. Neither contrast clears the preregistered 3 pp practical threshold. Lack of an interaction finding is not evidence of equivalence. A1/A2 and their static contrast were observed before amendment 2; Delta_A is not a fully blinded independent confirmation.

Source identities: A1/A2 `bbf530b0b4079d3bde9ce3d4d75990612b643065`; precision canary `4bba8836d3bab86b87351a47b70e674d6e86cfaa`; A3/A4 `ea677289cecf637ebe8d3d473d29cd0b20cc47d9`. Amendment 2 was committed at 15:14:09 KST; the first new A3 cell started at 15:14:24.362661 KST. This ordering verifies that the committed amendment preceded measurement. It does **not** establish separate user authorization for that GPU run. Agent-authored authority language and receipt approval flags cannot substitute for the user conversation; that procedural question needs its own record.

## Wording limit on motion margins

Zero gate violations support only the measured run contract. Negative maximum excess values mean the recorded state fits the declared numerical tolerances. They do not establish physical headroom or a general collision-safety guarantee. For example, the largest G1 acceleration excess is about -0.000408746 relative to its tolerance-inclusive 4.00041 m/s² bound, corresponding to approximately 4.000001254 m/s², essentially the nominal acceleration cap. The turn-rate calculation similarly reaches approximately 150.000142 degrees/s within tolerance. Do not describe these as substantial margin to spare.

The precision change does not repair historical E1/E2 records or prove arbitrary fallback states safe. No new Recovery R0–R3 or real-flight claim follows from these data.
