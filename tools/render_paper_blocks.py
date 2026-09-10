#!/usr/bin/env python3
"""Paper-style redrawing of existing documentation; no algorithm changes.

Run with detector_runs/venv/bin/python tools/render_paper_blocks.py.
Uses the existing Chrome exporter (websocket-client required).
"""
import hashlib
import json
import zipfile
from html import escape
from pathlib import Path
import render_presentation_figures as export

OUT = Path(__file__).resolve().parents[1] / 'docs/assets/paper'


class Diagram:
    def __init__(self, title, desc):
        self.s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="900" viewBox="0 0 1600 900" role="img" aria-labelledby="title desc"><title id="title">{escape(title)}</title><desc id="desc">{escape(desc)}</desc><defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10Z" fill="#253341"/></marker></defs><rect width="1600" height="900" fill="white"/><g font-family="Arial, Noto Sans CJK KR, sans-serif" fill="#16232f">']
        self.text(55, 68, title, 35, bold=True)

    def text(self, x, y, t, size=25, bold=False, anchor='start', color='#253341'):
        self.s.append(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}" fill="{color}">{escape(t)}</text>')

    def block(self, x, y, w, h, title, rows=(), fill='white'):
        self.s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="3" fill="{fill}" stroke="#253341" stroke-width="2"/>')
        self.text(x+w/2, y+43, title, 27, True, 'middle')
        for i, row in enumerate(rows):
            self.text(x+w/2, y+83+i*33, row, 23, anchor='middle')

    def edge(self, points, dashed=False):
        d = 'M' + ' L'.join(f'{x} {y}' for x, y in points)
        self.s.append(f'<path d="{d}" fill="none" stroke="#253341" stroke-width="2.5" marker-end="url(#arrow)"' + (' stroke-dasharray="8 6"' if dashed else '') + '/>')

    def note(self, lines):
        self.s.append('<path d="M55 784H1545" stroke="#b6c0c8"/>')
        for i, line in enumerate(lines):
            self.text(55, 824+i*35, line, 22, color='#52606d')

    def save(self, name):
        (OUT / name).write_text('\n'.join(self.s) + '\n</g></svg>\n', encoding='utf-8')


def perception():
    d = Diagram('Streaming perception pipeline', 'Existing documented image-processing branches, temporal buffer and frame-local selection output. No camera-to-policy integration is implied.')
    d.block(55, 310, 215, 150, 'Frame input', ['BGR image', 'sequence · time'], '#f1f4f6')
    d.block(365, 170, 260, 150, 'Frozen detector', ['Per-frame candidates', 'Top-K, K = 5'], '#eaf1f8')
    d.block(365, 475, 260, 150, 'Optical flow', ['Previous / current', 'grayscale frames'], '#f1f4f6')
    d.edge([(270, 355), (315, 355), (315, 245), (365, 245)])
    d.edge([(315, 355), (315, 550), (365, 550)])
    d.text(380, 145, 'GPU branch', 22)
    d.text(380, 455, 'CPU branch', 22)
    d.block(715, 310, 250, 175, 'Feature assembly', ['Candidates + GMC', 'Motion features'], '#eaf1f8')
    d.edge([(625, 245), (665, 245), (665, 360), (715, 360)])
    d.edge([(625, 550), (665, 550), (665, 435), (715, 435)])
    d.text(652, 680, 'Concurrent branches join after detection', 23, anchor='middle')
    d.block(1050, 310, 205, 175, 'History buffer', ['T = 16 frames', 'Sequence-local'])
    d.edge([(965, 397), (1050, 397)])
    d.block(1340, 310, 205, 175, 'Transformer', ['Temporal selector', 'P7c v2'], '#eaf1f8')
    d.edge([(1255, 397), (1340, 397)])
    d.block(1165, 610, 380, 115, 'Selection output', ['Candidate rank or NO_LOCK'], '#f1f4f6')
    d.edge([(1442, 485), (1442, 610)])
    d.text(1080, 550, 'State reset on sequence change', 21)
    d.note(['Solid arrows: implemented data flow. GT annotations are evaluation-only, not inference inputs.',
            'The output is frame-local candidate selection, not a physical track ID or a validated metric-range estimate.'])
    d.save('perception-block-diagram.svg')


def safety():
    d = Diagram('Safety filter: direction-preserving speed governor', 'Redrawing of the documented straight-corridor baseline. The policy command branches into direction selection and magnitude scaling. This is not the arc-filter variant.')
    d.text(55, 112, 'Straight-corridor baseline · existing simulation contract', 24, color='#52606d')
    d.block(55, 195, 260, 145, 'LiDAR input', ['Associated returns', 'Documented baseline'], '#f1f4f6')
    d.block(440, 195, 310, 145, 'Corridor selection', ['Along command direction', 'Forward clearance'], '#eaf1f8')
    d.block(890, 195, 280, 145, 'Speed-cap law', ['Configured mode', 'Scalar speed limit'], '#eaf1f8')
    d.edge([(315, 268), (440, 268)])
    d.edge([(750, 268), (890, 268)])
    d.text(781, 245, 'clearance', 20)
    d.block(55, 495, 260, 145, 'Policy command', ['Horizontal velocity', 'Requested direction'], '#f1f4f6')
    d.block(890, 495, 280, 145, 'Magnitude scaling', ['Direction unchanged', 'Speed bounded by cap'], '#eaf1f8')
    d.block(1290, 495, 255, 145, 'Filtered command', ['Horizontal velocity', 'To fixed controller'], '#f1f4f6')
    d.edge([(315, 567), (890, 567)])
    d.text(610, 600, 'Original horizontal command', 23, anchor='middle')
    d.edge([(375, 567), (375, 405), (595, 405), (595, 340)])
    d.text(406, 385, 'direction', 22)
    d.edge([(1030, 340), (1030, 495)])
    d.text(1050, 421, 'speed cap', 22)
    d.edge([(1170, 567), (1290, 567)])
    d.text(55, 727, 'Scope: horizontal magnitude only; no direction replanning and no vertical-command regulation.', 24)
    d.note(['This figure depicts the historical straight-corridor path, not the later arc-geometry comparison.',
            'Target-return association is detector-dependent in this baseline; collision avoidance is not guaranteed.'])
    d.save('safety-filter-block-diagram.svg')


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    perception()
    safety()
    entries = [{'svg': name+'.svg', 'png': name+'.png'} for name in
               ['perception-block-diagram', 'safety-filter-block-diagram']]
    export.OUT = OUT
    export.rasterize(entries)
    captions = '''# 논문형 블록 다이어그램

결과 요약 카드가 아니라 모듈의 입출력과 처리 흐름을 보여주는 그림입니다.
영문 라벨·흰 배경·단순 박스와 화살표로 구성했습니다. SVG는 벡터 원본, PNG는 3840×2160입니다.

## Fig. 1. Streaming perception pipeline

RGB 입력에서 검출과 광류를 병렬 계산한 뒤 후보·motion feature를 결합하여 시간 이력과 Transformer 선택기에 전달합니다.
출력은 프레임 안의 후보 rank 또는 NO_LOCK입니다. 물리적 track ID나 거리·자세 추정 출력을 뜻하지 않습니다.
GT는 평가용이며 추론 입력에 포함되지 않습니다. 카메라→PPO 실기 통합은 그리지 않았습니다.

근거: [Streaming perception v1](../../specs/perception_streaming_v1.md).

## Fig. 2. Direction-preserving speed governor

LiDAR 처리 경로와 정책 명령 경로를 분리했습니다. 정책 명령의 방향은 회랑 선택에, 원래 명령은 크기 조정에 전달됩니다.
선택된 clearance로 얻은 cap은 수평 명령의 크기만 제한합니다. 방향 재계획이나 수직 제어 블록은 아닙니다.
이 그림은 기존 **직선 회랑 기준선**입니다. 후속 arc variant를 같은 구조로 묶지 않았습니다.

근거: [기존 상세 그림](../motar-safety-filter.svg) 및 README의 safety-filter 설명.

재생성: `python tools/render_paper_blocks.py` (Chrome, websocket-client 필요).
새 실험이나 알고리즘 변경 없이 기존 문서만 재도식화했습니다.
'''
    (OUT / 'README.md').write_text(captions, encoding='utf-8')
    files = [OUT / e[k] for e in entries for k in ('svg', 'png')] + [OUT / 'README.md']
    (OUT / 'manifest.json').write_text(json.dumps({p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}, indent=2)+'\n')
    with zipfile.ZipFile(OUT / 'motar-paper-block-diagrams.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for p in files + [OUT / 'manifest.json']:
            z.write(p, p.name)
    cards = ''.join(f'<h2>{title}</h2><img src="{e["svg"]}" alt="{title}"><p><a href="{e["svg"]}" download>SVG</a> · <a href="{e["png"]}" download>4K PNG</a></p>' for title,e in zip(['Perception pipeline', 'Safety filter — straight-corridor baseline'], entries))
    (OUT / 'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MOTAR 논문형 블록 다이어그램</title><style>body{max-width:1400px;margin:32px auto;padding:0 20px;font:18px/1.6 system-ui;color:#253341}img{width:100%;height:auto}a{color:#25608c}</style><h1>논문형 블록 다이어그램</h1><p><a href="motar-paper-block-diagrams.zip">전체 다운로드</a> · <a href="README.md">그림 설명과 근거</a></p>'+cards+'</html>\n', encoding='utf-8')
    print('Built two paper-style SVG + 4K PNG diagrams')


if __name__ == '__main__':
    build()
