"""Validate EQ-0 document/evidence/source bindings. Read-only; no simulator or GPU imports.

Use --full-raw for all 3,347 preserved raw members. PASS here validates this audit package's
integrity and internal consistency, not the physical environment's failed/untested gates.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def git(*args, cwd=REPO):
    return subprocess.check_output(['git', '-C', str(cwd)] + list(args), text=True)


def validate(full_raw=False):
    index = json.loads((HERE / 'index.json').read_text())
    actual = {str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file()}
    assert actual == set(index['files']) | {'index.json'}, 'Unbound or missing audit files'
    for name, digest in index['files'].items():
        assert sha(HERE / name) == digest, name
    source = json.loads((HERE / 'source_bindings.json').read_text())
    for name, record in source['audited_files'].items():
        measured = subprocess.check_output(['git', '-C', str(REPO), 'show',
                                             source['measurement_source'] + ':' + name])
        assert hashlib.sha256(measured).hexdigest() == record['sha256'], name
        assert (REPO / name).read_bytes() == measured, name
    for name in ['ceedc52dd9d057a5e5958594076c1bcdcf8efb69',
                 '7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2']:
        subprocess.run(['git', '-C', str(REPO), 'merge-base', '--is-ancestor', name, 'HEAD'], check=True)
    links = 0
    for file in HERE.rglob('*.md'):
        # Local path existence only; no external network or heading-anchor validation.
        body = re.sub(r'```.*?```', '', file.read_text(), flags=re.S)
        for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)', body):
            target = target.strip('<>')
            if urlsplit(target).scheme or target.startswith('#'):
                continue
            assert (file.parent / unquote(target.split('#')[0])).exists(), (file, target)
            links += 1
    inventory = json.loads((HERE / 'supporting/g4/run_inventory.json').read_text())
    states = json.loads((HERE / 'supporting/g1/record_forensics.json').read_text())
    flags = json.loads((HERE / 'supporting/g3/reproduced_checks.json').read_text())
    assert inventory['cells'] == flags['frames_sha_checked_cells'] == 112
    assert inventory['ledger_counts']['quota'] == 57344
    assert inventory['ledger_counts']['nonprimary_tail'] == 29042
    assert sum(g['frames'] for g in states['groups'].values()) == 2320985
    assert sum(g['frames'] for g in flags['groups'].values()) == 2320985
    assert len(inventory['quota_vs_legacy_result_differences']) == 96
    for prefix, expected in [('quota', (55441, 1898, 5)), ('legacy_result', (55478, 1997, 0))]:
        assert tuple(inventory['outcome_populations'][prefix + '/' + key]
                     for key in ['capture', 'crash', 'timeout']) == expected
    raw = Path(inventory['raw_root'])
    launch = json.loads((raw / 'launch_receipt.json').read_text())
    audit_text = (HERE / 'EQ0_ENVIRONMENT_AUDIT.md').read_text()
    assert launch['source_commit'] == source['measurement_source']
    assert launch['source_tree'] in audit_text
    for name, digest in inventory['evidence_file_sha256'].items():
        assert sha(raw / name) == digest, name
    baseline = json.loads((HERE / 'supporting/g4/before.json').read_text())
    main = REPO.parent / 'aerial_gym_simulator'
    assert git('rev-parse', 'HEAD', cwd=main).strip() == baseline['main_head']
    for name, digest in baseline['main_wip_sha256'].items():
        assert sha(main / name) == digest, name
    assert git('stash', 'list', '--format=%H %gd %gs') == baseline['stash']
    checkpoint = (REPO / 'aerial_gym/rl_training/rl_games/runs/'
                  'ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/'
                  'last_gen_ppo_ep_25000_rew_39.742134.pth')
    assert sha(checkpoint) == launch['checkpoint_sha256']
    verified_raw = 0
    if full_raw:
        prior = json.loads((REPO / 'docs/evidence/occ_p2_pre_execution_2026-10-17/preservation.json').read_text())
        bindings = dict(prior['manifests'])
        preserved_manifest = REPO / 'docs/evidence/occ_p2_confirmatory_2026-10-17/RAW_MANIFEST.json'
        bindings[str(raw / 'RAW_MANIFEST.json')] = {'sha256': sha(preserved_manifest)}
        for name, record in bindings.items():
            manifest = Path(name)
            assert sha(manifest) == record['sha256'], name
            for relative, digest in json.loads(manifest.read_text())['files'].items():
                assert sha(manifest.parent / relative) == digest, relative
                verified_raw += 1
        assert verified_raw == 3347
    return {'status': 'PASS', 'scope': 'EQ-0 audit integrity, not environment qualification',
            'audit_files_bound': len(index['files']), 'measured_source_files': len(source['audited_files']),
            'local_links_checked': links, 'cells': 112, 'quota_episodes': 57344,
            'primary_frames': 2320985, 'bound_metadata_files_checked': len(inventory['evidence_file_sha256']),
            'raw_members_verified_this_invocation': verified_raw,
            'main_wip_stash_policy_integrity': 'PASS', 'prior_commits_retained': True,
            'environment_modified': False, 'simulator_or_gpu_executed': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full-raw', action='store_true')
    print(json.dumps(validate(parser.parse_args().full_raw), indent=2, sort_keys=True))
