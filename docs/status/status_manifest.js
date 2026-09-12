'use strict';
function statusLines(manifest) {
  if (manifest.schema_version !== 1 || manifest.real_flight_validated !== false) {
    throw new Error('Unsupported status manifest');
  }
  return Object.entries(manifest.track_d).map(([id, row]) => `${id} · ${row.label}: ${row.status}`);
}
if (typeof module !== 'undefined') module.exports = { statusLines };
if (typeof document !== 'undefined') {
  fetch('../status_manifest.json').then(response => {
    if (!response.ok) throw new Error('Status manifest unavailable');
    return response.json();
  }).then(manifest => {
    const node = document.getElementById('public-status-manifest');
    node.textContent = statusLines(manifest).join(' · ');
  }).catch(() => {
    document.getElementById('public-status-manifest').textContent =
      'Status unavailable — consult VERIFICATION.md; no PASS inferred.';
  });
}
