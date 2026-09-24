# Design system

One visual language for the public site (`docs/status/`) and the paper figures
(`docs/assets/paper/final/`). The target is a strong robotics or computer-vision project page:
academic, precise, calm. Information density is high and visual noise is low. The site implements
this file in `docs/status/site.css`; the figures implement it in `tools/build_paper_figures.py`.

The page must not read as a product landing page, a dashboard or a template. There are no gradients,
glass effects, neon, large rounded cards, heavy shadows or decorative animation.

## Colour

| Token | Hex | Use |
| --- | --- | --- |
| `--bg` | `#fbfaf7` | Page background (warm white) |
| `--surface` | `#ffffff` | Figure panels, tables |
| `--ink` | `#1f1f1f` | Body text, headings, key numbers |
| `--ink-2` | `#4a4a4f` | Secondary text, arrows, table headers |
| `--muted` | `#6e6e73` | Notes, captions' secondary lines, axis text. Never on `--blue-wash` (4.34:1) |
| `--rule` | `#dedad2` | Hairline borders and table rules |
| `--rule-strong` | `#cfcbc3` | Figure boxes, axis spines |
| `--blue` | `#2f6599` | Research blue: data marks, active states |
| `--link` | `#245180` | Link text (7.85:1 on `--bg`) |
| `--blue-wash` | `#e8eef6` | Quiet highlight behind a row or band |
| `--accent` | `#c2562b` | Restrained accent: borders and marks only, for attention labels (MATERIAL_LOSS, CAUSALITY NOT TESTED) and the one arm a figure is about. Never text (4.32:1) |
| `--teal` | `#1c9a7b` | Third data category, only when three categories must be told apart |
| `--ref` | `#8e8e93` | Reference or baseline marks (arm H, the analytic observation) |

Rules:

- **One accent per view.** A section uses the accent at most once. Most figures use blue and grey
  only.
- **Colour is never the only carrier of meaning.** Every verdict is written as a word (INCONCLUSIVE,
  MATERIAL_LOSS, NOT_TESTED), and every category has a label or a legend.
- **Text wears text tokens.** Values and labels are `--ink`, `--ink-2` or `--muted`, never a data
  colour.
- **Verdict tags are hairline boxes with uppercase text,** not filled pills. An accent border marks a
  loss or an untested cause; everything else uses `--ink-2`.
- The three data colours (blue, accent, teal) pass the dataviz palette validator for a light surface
  on all pairs: lightness band, chroma floor, CVD separation ≥ 8 and a normal-vision floor ≥ 15.

The site is light-only by design: it is a paper page, and its figures are drawn on white.

## Typography

No web fonts are loaded. The page renders with system fonts, so it is private, fast and reproducible
offline.

| Role | Stack | Size |
| --- | --- | --- |
| Headings | `Georgia, "Times New Roman", "Noto Serif", serif` | H1 44–56 px · H2 28 px · H3 19 px |
| Body | `system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", Arial, "Noto Sans", "Apple SD Gothic Neo", sans-serif` | 17 px / 1.6 (16 px under 760 px) |
| Captions, tables | body stack | 14–15 px |
| Technical tokens only | `ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace` | 0.88 em |

- No scientific text below 13 px. The smallest caption and footnote size is 13 px, and there are no
  9 px captions.
- Key numbers use the body sans at 600 weight with proportional figures. Tables use `tabular-nums`.
- `pp` is always written in full once per page: "pp = percentage points".
- Negative numbers use the true minus sign (−), not a hyphen.

## Layout

- Prose column 720 px (about 70 characters); figures up to 1040 px; side gutter at least 20 px.
- Breakpoints: 760 px (single column, figure scroll), 1100 px (wide figures).
- Section rhythm: 72 px between sections on desktop, 48 px on mobile. Each section is one question,
  one figure, one short interpretation and one limitation.
- Numbered section labels (`01`–`08`) in `--muted`, above a serif H2.
- Tables and wide figures scroll horizontally inside their own container on narrow screens. The page
  body never scrolls sideways.

## Components

| Component | Specification |
| --- | --- |
| Top navigation | Sticky, `--bg`, 1 px `--rule` bottom border, text links only; six primary links |
| Hero | Title (serif), expansion, one-sentence thesis, Figure 1, three result call-outs, one boundary line |
| Result call-out | Large number (600 weight), one-line label, one qualifier, the source record. No card background: a 2 px `--rule-strong` top rule only |
| Figure | White panel, 1 px `--rule` border, figcaption: **Figure N.** + one-sentence claim + one limitation line + an `SVG` link (PNG and PDF are rebuilt by the generator) |
| Limitation box | 3 px `--ink-2` left rule, `--surface` background, 15 px text |
| Verdict tag | Uppercase 12.5 px, 600 weight, 1 px border (`--ink-2`, or `--accent` for a loss or an untested cause), 2 px radius at most |
| Buttons | Only inside the browser preview; flat, 1 px border |

Corner radius is 0–2 px everywhere. Shadows are not used. Motion is limited to the browser preview
and stops under `prefers-reduced-motion`.

## Accessibility

- Semantic landmarks (`header`, `nav`, `main`, `section`, `footer`), a skip link and one `h1`.
- Every figure has alt text of at least 15 characters. Its SVG carries `<title>` and `<desc>` with
  the key numbers.
- Visible focus: 2 px `--blue` outline with a 2 px offset.
- AA contrast for all text, measured above.
- The page is complete without JavaScript. Status cells fall back to the machine-readable records.

## Figures

Every paper figure comes from `tools/build_paper_figures.py`, in one family.

| Property | Rule |
| --- | --- |
| Canvas | Figures 1–6: 7 in wide, the printed width of a full two-column page, so point sizes in the files are printed sizes. The system map is a 10 in screen diagram. White. SVG (text kept as text) is tracked; PDF (fonts embedded) and PNG (220 dpi) are rebuilt byte-exactly from the same figure object and pinned by hash |
| Font | Liberation Sans (Arial metrics); SVG falls back to Arial and Helvetica. File names in Liberation Mono |
| Sizes (printed) | Panel letter 10 pt bold · panel title 9 pt bold · labels and ticks 8 pt · headline values 9–11 pt · notes and tags 7.5 pt · nothing under 7 pt (`tests/test_paper_figures_final.py`) |
| Panels | Lowercase bold letter (a, b, c, d) and a short title, top-left, on one shared baseline |
| Lines | Axes 0.7 pt in `--rule-strong`, gridlines 0.6 pt in `#ebe8e2` (solid, never dashed), data 1.1–2.2 pt |
| Marks | Dots at least 5 pt with a white ring; open dots for seeds, filled dots for means; bars for intervals |
| Boxes | Square corners, 0.8 pt border, no fill or `#f7f6f2` for schematic panels |
| Arrows | Straight, `-|>`, 1 pt, `--ink-2` |
| Colour | As above; the accent marks at most one element per figure |
| Numbers | Read from canonical records (see `SOURCES` in the generator); every drawn value is stored in `manifest.json` |
| Schematics | Labelled "schematic" in the panel title or the panel itself; never presented as recorded imagery |
| Accessibility | SVG `role="img"`, `<title>` and `<desc>` containing the headline values and limits |

**Captions** on the site follow the same order every time: the claim, then the evidence (n, interval),
then the limit. Example: *"Target motion was not a monotonic difficulty ladder. Seed-paired
differences vs the training target; unit of inference: evaluation seed, n = 3. Association only; frozen
policy F, one arena."*

**Do not** fabricate images or data, draw recorded-looking scenes by hand, add significance stars,
use dual axes, stretch an axis to exaggerate a gap without saying so, or show a performance
leaderboard against published systems.
