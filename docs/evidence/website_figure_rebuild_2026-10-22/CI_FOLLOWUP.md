# Publication redesign — CI path-contract follow-up

The site and all twenty publication artifacts from `f9c1772831936bf59f0f97b91f09f514baf794f7`
were deployed and verified byte-for-byte over HTTPS. The Pages deployment succeeded.

The separate CPU workflow failed in
`test_public_docs_consistency.PublicDocsTest.test_all_nine_figures_preserved_off_landing_page`:
its README-image assertion allowed only `docs/assets/paper/final/`, the pre-redesign folder.
Run: https://github.com/joshualikaist/MOTAR/actions/runs/39804319643

The test now checks the approved new family and binds each README image to its actual
SHA-256 in the publication manifest. The original two-image README limit, nine historical
overview figures and archived README preservation checks are retained. No scientific
criterion, runtime, figure, page or measurement is changed by this follow-up.

Validation: all nineteen `test_public*.py` checks pass; the expanded documentation/figure
suite is 177 tests, zero failures/errors, in pinned Python 3.8.20. Full receipt:
`ci_followup_tests.json`. Historical figure byte/number checks require their pinned
Python/plotting environment; the default Python 3.13 summation differs at float precision.
The research source and Main WIP/stash remain untouched.
