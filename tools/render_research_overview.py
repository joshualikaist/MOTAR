#!/usr/bin/env python3
"""Generate a NEW dated explanatory figure set; never rewrite pinned paper assets.

SVG is the source of truth. --export derives PNG/vector PDF using the existing
Chrome exporter; no simulator imports or experiment execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
import render_paper_blocks as blocks
import render_presentation_figures as exporter

OUT = Path(__file__).resolve().parents[1] / 'docs/assets/paper/overview-2026-09-13'
SENSOR = '#edf4f6'
LEARNED = '#e7edf9'
FIXED = '#f2f2ed'


def diagram(stem, title, desc, nodes, edges, notes):
    d = blocks.Diagram(title, desc)
    for x, y, label, rows, category, future in nodes:
        d.block(x, y, 425, 140, label, rows, category, planned=future)
    for points, future in edges:
        d.edge(points, dashed=future)
    d.note(notes)
    d.save(stem + '.svg')


def chain(stem, title, labels, notes):
    positions = [(110, 170), (1060, 170), (1060, 375), (110, 375), (110, 580), (1060, 580)]
    nodes = [(x, y, *item) for (x, y), item in zip(positions, labels)]
    routes = [[(535,240),(1060,240)], [(1272,310),(1272,375)],
              [(1060,445),(535,445)], [(322,515),(322,580)], [(535,650),(1060,650)]]
    edges = [(route, labels[i+1][-1]) for i, route in enumerate(routes[:len(labels)-1])]
    diagram(stem, title, 'Explanatory module map of existing research, not a deployment design.', nodes, edges, notes)


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    blocks.OUT = OUT
    chain('research-overview-block-diagram', 'Research question: sensing in clutter', [
        ('Simulation scene', ['Moving object · obstacles'], SENSOR, False),
        ('Sensor observations', ['Camera + LiDAR'], SENSOR, False),
        ('Perception / tracking', ['Detection · association'], LEARNED, False),
        ('Navigation study', ['Clutter · temporal context'], LEARNED, False),
        ('Historical task outcomes', ['Approach / capture records'], FIXED, False),
        ('Research evidence', ['Errors · limits · provenance'], SENSOR, False)],
        ['Research questions, not a single integrated real-image-to-flight pipeline.',
         'Simulation only. Historical interception / capture labels retain their original meaning.'])
    chain('system-control-block-diagram', 'Documented simulation control stack', [
        ('Camera + LiDAR', ['Existing simulation sensing'], SENSOR, False),
        ('Structured observation', ['Existing detector / tracker'], FIXED, False),
        ('Transformer PPO', ['Policy history encoding'], LEARNED, False),
        ('Safety filter', ['Configured diagnostic arm'], FIXED, False),
        ('Velocity controller', ['Fixed control stack'], FIXED, False),
        ('Rigid-body simulation', ['Physics state update'], FIXED, False)],
        ['High-level description of existing modules; filter activation is lineage-specific.',
         'The real-image temporal selector is a separate pipeline, not this simulator detector.'])
    diagram('perception-tracking-block-diagram', 'Real-image perception: separate evidence lineage',
        'Detector and optical flow run in parallel; selection does not imply metric state or policy integration.', [
        (110,170,'RGB frame',['Sequence · timestamp'],SENSOR,False),
        (1060,170,'Detector',['Top-K candidates'],LEARNED,False),
        (110,375,'Optical flow',['Inter-frame motion'],FIXED,False),
        (1060,375,'Feature history',['Candidates + motion'],FIXED,False),
        (1060,580,'Temporal association',['Candidate rank / no selection'],LEARNED,False),
        (110,580,'Policy observation link',['Not integrated here'],FIXED,True)], [
        ([(535,240),(1060,240)],False), ([(322,310),(322,375)],False),
        ([(1272,310),(1272,375)],False), ([(535,445),(1060,445)],False),
        ([(1272,515),(1272,580)],False), ([(1060,650),(535,650)],True)],
        ['Separate measured-error branch: real data → error model → simulation injection (P8–P10).',
         'No GT identifiers at inference. Dashed link: unimplemented integration, not an established state estimator.'])
    chain('observation-transformer-block-diagram', 'Observation and temporal policy: historical contract', [
        ('Sensor-derived features',['Scan · object · ego histories'],SENSOR,False),
        ('Structured history',['Canonical field contract'],FIXED,False),
        ('Token encoders',['Separate feature families'],LEARNED,False),
        ('Temporal Transformer',['Policy context representation'],LEARNED,False),
        ('Actor output',['Simulation action interface'],LEARNED,False),
        ('Recorded rollout',['Evaluation evidence'],SENSOR,False)],
        ['Policy Transformer and real-image association Transformer are different models.',
         'Privileged GT is excluded from actor input; this diagram does not specify a new policy.'])
    diagram('safety-filter-block-diagram', 'Safety-filter diagnosis: comparison structure',
        'Descriptive comparison map, not a new filter or guaranteed collision avoidance.', [
        (110,170,'Policy command',['Existing simulation action'],LEARNED,False),
        (1060,170,'Sensor context',['LiDAR · ego state'],SENSOR,False),
        (110,375,'Geometry comparison',['Corridor / arc alternatives'],FIXED,False),
        (1060,375,'Filter configuration',['Separate cap-law comparison'],FIXED,False),
        (110,580,'Recorded outcomes',['Historical simulation runs'],SENSOR,False),
        (1060,580,'Failure diagnosis',['Geometry · limits · uncertainty'],SENSOR,False)], [
        ([(322,310),(322,375)],False), ([(1272,310),(1272,375)],False),
        ([(322,515),(322,580)],False), ([(1272,515),(1272,580)],False),
        ([(535,650),(1060,650)],False)],
        ['Geometry and cap law are comparison axes; alternatives are not sequential filters.',
         'Historical diagnosis is not a safety certificate or real-flight validation.'])
    diagram('appearance-rendering-block-diagram', 'Appearance engineering: preserve the boundary',
        'Production proxy and independent rendering studies are separate; shadow output has no downstream consumer.', [
        (110,170,'Analytic proxy',['Existing sensor path'],FIXED,False),
        (110,375,'Existing detector',['Unchanged observations'],FIXED,False),
        (1060,170,'Visual geometry',['Independent asset lineage'],FIXED,False),
        (1060,375,'Appearance studies',['Normal · material · lighting'],FIXED,False),
        (1060,580,'Shadow query',['Cost / non-interference only'],SENSOR,False),
        (110,580,'Perception integration',['Not started'],FIXED,True)], [
        ([(322,310),(322,375)],False), ([(1272,310),(1272,375)],False)],
        ['Shadow results are diagnostic buffers only: no arrow into the production detector.',
         'Shading, feasibility and shadow cost are different experiments; shortcut reduction is unmeasured.'])
    chain('evidence-reproducibility-block-diagram', 'Evidence before claims', [
        ('Preregister',['Question · criteria · scope'],FIXED,False),
        ('Run',['Pinned source · environment'],FIXED,False),
        ('Receipt',['Seeds · parameters · hashes'],SENSOR,False),
        ('Independent checks',['Recompute · regression'],FIXED,False),
        ('Bounded claim',['Result with limitations'],SENSOR,False),
        ('Public record',['Negative / withdrawn retained'],SENSOR,False)],
        ['A passing software test is not a positive scientific result.',
         'Public overview links the evidence; it does not replace original receipts.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export', action='store_true')
    args = parser.parse_args()
    build()
    if args.export:
        exporter.OUT = OUT
        exporter.rasterize([dict(svg=p.name, png=p.stem+'.png', pdf=p.stem+'.pdf')
                            for p in sorted(OUT.glob('*.svg'))])
    files = sorted(p for p in OUT.iterdir() if p.suffix in ('.svg','.png','.pdf'))
    manifest = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    with zipfile.ZipFile(OUT/'research-overview-figures.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for p in files + [OUT/'manifest.json']:
            archive.write(p, p.name)
    print(json.dumps({'figures':len(list(OUT.glob('*.svg'))), 'artifacts':len(files)}))


if __name__ == '__main__':
    main()
