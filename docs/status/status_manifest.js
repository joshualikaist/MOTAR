'use strict';
function statusLines(manifest) {
  if (manifest.schema_version !== 2 || manifest.real_flight_validated !== false ||
      Object.keys(manifest.track_d).join(',') !== 'D1,D2,D3,D4,D5,D6,D7,D8,D9') {
    throw new Error('Unsupported status manifest');
  }
  return Object.entries(manifest.track_d).map(([id, row]) => `${id} · ${row.label}: ${row.status}`);
}
function renderStatus(manifest, node) {
  statusLines(manifest);
  const list = document.createElement('dl');
  list.className = 'evidence-list';
  for (const [id, row] of Object.entries(manifest.track_d)) {
    if (!/^(docs|results)\/[a-zA-Z0-9_./-]+\.md$/.test(row.evidence) || row.evidence.includes('..')) {
      throw new Error('Unsafe evidence path');
    }
    const item = document.createElement('div');
    const label = document.createElement('dt');
    const link = document.createElement('a');
    link.href = '../../' + row.evidence;
    link.textContent = `${id} · ${row.label}`;
    label.appendChild(link);
    const state = document.createElement('dd');
    state.textContent = row.status;
    item.append(label, state); list.appendChild(item);
  }
  node.replaceChildren(list);
}
if (typeof module !== 'undefined') module.exports = { statusLines, renderStatus };
if (typeof document !== 'undefined') {
  fetch('../status_manifest.json').then(response => {
    if (!response.ok) throw new Error('Status manifest unavailable');
    return response.json();
  }).then(manifest => renderStatus(manifest, document.getElementById('public-status-manifest')))
    .catch(() => {
      document.getElementById('public-status-manifest').textContent =
        'Status unavailable — consult VERIFICATION.md; no PASS inferred.';
    });
}
