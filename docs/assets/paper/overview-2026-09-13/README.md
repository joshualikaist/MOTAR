# Research overview figures — 2026-09-13

New explanatory block diagrams; no algorithm changes or new performance results.
SVG is authoritative. Each same-stem PNG is 3840×2160; PDF is exported from vector SVG.

1. [Research question](research-overview-block-diagram.svg)
2. [Existing simulation control stack](system-control-block-diagram.svg)
3. [Separate real-image perception pipeline](perception-tracking-block-diagram.svg)
4. [Observation and policy Transformer](observation-transformer-block-diagram.svg)
5. [Safety-filter diagnosis](safety-filter-block-diagram.svg)
6. [Appearance/rendering boundaries](appearance-rendering-block-diagram.svg)
7. [Evidence and reproducibility](evidence-reproducibility-block-diagram.svg)

[Download all 21 artifacts and hashes](research-overview-figures.zip).
Blue denotes learned modules, warm neutral fixed/control, light blue-gray sensing/evidence.
Dashed borders/arrows denote unimplemented links. Historical task names are retained;
none of these figures establishes real-flight or integrated real-image-to-policy performance.

Rebuild: `python tools/render_research_overview.py --export` from the repository root,
with Chrome and websocket-client installed. Existing parent-directory figures and their
hash-pinned packages are untouched. The manifest covers SVG/PNG/PDF, not this README.
