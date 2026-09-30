# MOTAR paper companion

The current overview is [MOTAR_overview_2026-09-26.pptx](MOTAR_overview_2026-09-26.pptx): 12 widescreen slides with Korean main sentences, English diagrams and complete Korean speaker notes. All slide text, plots and diagrams are native editable PowerPoint objects. The [paper Markdown](../paper/MOTAR_draft_2026-09-26.md) is the source for the [Word review draft](../paper/MOTAR_draft_2026-09-26.docx).

The original Word and 25-slide PowerPoint are preserved outside this repository. The new documents replace their research story with the independent n=7 replication and its limits. The filename date identifies the requested evidence edition, not the time of the latest authoring pass.

Create a separate authoring environment; do not modify the simulator environment:

```bash
python3 -m venv /tmp/motar-authoring
/tmp/motar-authoring/bin/pip install -r presentations/requirements-authoring.txt
/tmp/motar-authoring/bin/python presentations/build_companion.py
/tmp/motar-authoring/bin/python presentations/build_companion.py --check
```

The recorded build uses CPython 3.13 on Linux with Cairo and Noto Sans CJK KR installed. LibreOffice can render the Office files for visual review. Fonts must be installed for equivalent Korean line wrapping.

Edit the English manuscript in Markdown and the deck content/layout in [build_companion.py](build_companion.py), then rebuild. Edits made directly in Word or PowerPoint remain editable but will be overwritten by the next generated build; port approved changes to the source first. The generator verifies the final-figure manifest's upstream evidence hashes, reads scientific values from that manifest, renders the six tracked SVGs into the Word document, and normalizes Office timestamps for reproducible checks. [companion_manifest.json](companion_manifest.json) records sources, output hashes, paper word count, and slide provenance.

The six paper figures are in [docs/assets/paper/final](../docs/assets/paper/final/). Their SVG sources, values and print exports are maintained by `tools/build_paper_figures.py`; rebuild those before the companion if a figure caption changes. No document command starts training or evaluation.

The experimental programme is closed. Remaining editorial tasks and prohibited claims are in the [paper checklist](../paper/PAPER_TODO_2026-09-26.md). Review the full diff before committing or publishing the new artifact links.
