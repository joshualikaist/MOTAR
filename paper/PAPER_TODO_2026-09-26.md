# MOTAR paper submission checklist

Evidence is closed at `2da5b12c1954ee768fcba59d1fb1fb9c78f7d536`. This checklist is editorial work, not authorization for experiments. The dated Markdown is the paper source; its DOCX and the 12-slide PPTX are review deliverables, not an accepted publication.

## Citations to complete

- [ ] Verify publisher metadata, DOI, page ranges and author spellings for references R1–R7; preserve preprint status where applicable. Use the primary links already in the draft.
- [ ] Add the exact upstream Aerial Gym publication/software version, Lee controller reference, and relevant sensor/renderer dependencies.
- [ ] Add primary dataset citations for ETH ds5 and NPS, including the precise sequence, split, permissions and acquisition route. Do not imply that real imagery is a live flight evaluation.
- [ ] Add foundational references for BCa bootstrap intervals, exact paired sign-flip tests and Holm correction. The frozen analysis remains the authority for the procedure actually executed.
- [ ] Verify affiliation, corresponding-author designation, acknowledgements and funding with the authors before submission. No identifier or grant is invented.

## Figures and exports

- [ ] Export the six final figures from `tools/build_paper_figures.py` at the venue's actual column width. SVG is tracked; PNG/PDF can be regenerated. Check embedded fonts, thin lines, minimum type size and print contrast.
- [ ] Inspect Figures 1–3 after final layout: the ETH range measurement is separate from P8→P9 injection; arc−riskcap changes geometry and stopping law together.
- [ ] Keep Figure 4 central, with all seven seed observations, current and historical campaigns separated, and the density limitation visible.
- [ ] Keep policy R distinct in Figure 5 and retain causality NOT_TESTED.
- [ ] Verify Figure 6 against the screened primary publications. Its marks indicate coverage, not an exhaustive novelty claim or ranking.
- [ ] Re-render the DOCX and PPTX after any author edits; the presentation contains editable native text, plots and diagrams, and complete speaker notes.

## Tables and statistical reporting

- [ ] Format Tables 1–4 for venue width. Keep inferential unit and denominator visible, use consistent pp notation, and align confidence limits.
- [ ] Preserve 7 fresh seeds, 112 cells and 229,376 primary episodes as different counts. The unit of inference is the paired evaluation seed.
- [ ] Keep BCa95 with each primary contrast. Exact p=0.015625 and Holm p=0.046875 lie at the seven-seed resolution boundary; retain this qualification next to the tests.
- [ ] Do not pool the original n=3 campaign with n=7. Keep the within-n=7 estimator diagnosis distinct from the between-campaign difference.
- [ ] Keep P9's episode-level interval, P10's training-seed interval and safety's cell-pooled/seed-level intervals explicitly labeled.
- [ ] Preserve minimum distance as WITHHELD_SEMANTIC_MISMATCH; no substitute distance statistic is authorized.

## External baseline gap

- [ ] State Class A matched external comparisons = 0. This is a scope limitation, not an experiment action item.
- [ ] Keep Elastic Tracker B0 as environmental reproduction failure, with B1 not started. Neither is a MOTAR benchmark.
- [ ] Resolve bibliographic/classification inconsistencies in the existing literature ledger by consulting primary sources; do not infer benchmark availability from a public repository alone.

## Venue and release preparation

- [ ] Confirm venue, page limit, language, anonymity policy and official template. The review DOCX intentionally does not reuse unrelated headers or metadata from the old document.
- [ ] Finalize abstract length, author block, keywords, figure placement, table typography and reference style.
- [ ] Add the required data/code availability statement: raw replication evidence is outside normal Git and not publicly downloadable; manifests and archive hash are distributed.
- [ ] Run PUBLIC_CPU, Node 20 site tests, GT browser validators, public-doc/schema checks, document/figure `--check`, link validation and `git diff --check` after final edits.
- [ ] Review the full diff before commit. Publish links to new artifacts become live only after an approved commit/push and site deployment.

## Claims that must not appear as conclusions

- State of the art, real-world readiness, real-flight validation, or sim-to-real validation.
- E2 universally easier, E2 improving at every density, or static targets inherently harder.
- Seven-seed tests described as highly significant, or 229,376 episodes described as independent replications.
- D8b causality proven, a specific renderer/detector defect established, or its effect ranked as a universal bottleneck across different policies.
- PPO retraining required; P10 instead remains INCONCLUSIVE.
- The ETH 6.2% range error injected into policy F; P9 injects a separate pixel/temporal model and retains analytic range/confidence.
- The −1.4903 pp arc−riskcap result as an isolated geometry effect, or the implemented speed filter as a safety guarantee.
- Validated minimum-distance results, persistent physical target identity, or a matched external baseline that has not been run.

No PPO training, extra seeds, D8c experiment, Elastic Tracker run, GPU evaluation, reward/controller/target change, or scientific-number change is part of this checklist.
