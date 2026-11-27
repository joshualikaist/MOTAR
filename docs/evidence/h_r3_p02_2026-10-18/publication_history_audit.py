#!/usr/bin/env python3
"""Read-only full unpublished Git history audit. No credential values are reported."""
import collections
import difflib
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1')
MAIN = Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator')
OUT = Path(sys.argv[1])
EXPECTED_REMOTE = 'git@github.com:joshualikaist/MOTAR.git'


def git(*args, root=ROOT, check=True):
    p = subprocess.run(['git', *args], cwd=root, capture_output=True, check=check)
    return p.stdout


def sha(data):
    return hashlib.sha256(data).hexdigest()


head = git('rev-parse', 'HEAD').decode().strip()
branch = git('branch', '--show-current').decode().strip()
remote_urls = {
    'fetch': git('remote', 'get-url', '--all', 'origin').decode().splitlines(),
    'push': git('remote', 'get-url', '--push', '--all', 'origin').decode().splitlines(),
}
assert remote_urls == {'fetch': [EXPECTED_REMOTE], 'push': [EXPECTED_REMOTE]}
env = dict(os.environ, GIT_TERMINAL_PROMPT='0', GIT_SSH_COMMAND='ssh -o BatchMode=yes -o StrictHostKeyChecking=yes')
remote = subprocess.run(['git', 'ls-remote', '--heads', 'origin', 'refs/heads/main', 'refs/heads/codex/mt-p1'],
                        cwd=ROOT, env=env, capture_output=True, timeout=45)
advertised = {}
if remote.returncode == 0:
    advertised = {line.split()[1]: line.split()[0] for line in remote.stdout.decode().splitlines()}
remote_base = advertised.get('refs/heads/codex/mt-p1')
assert remote_base == 'fbee900b0d1483bdc336716eb6e57180e06deebb', 'Unexpected remote branch advancement; stop publication.'
assert advertised.get('refs/heads/main') == '2da5b12c1954ee768fcba59d1fb1fb9c78f7d536', 'Remote main changed; inspect before publication.'
cached_base = git('rev-parse', 'refs/remotes/origin/codex/mt-p1').decode().strip()
assert remote_base == cached_base, 'Remote research branch differs; audit exact remotely advertised base before publication.'
range_args = [head, '^' + remote_base]
commits = git('rev-list', '--reverse', *range_args).decode().splitlines()
allremote_commits = git('rev-list', head, '--not', '--remotes').decode().splitlines()
assert set(commits) == set(allremote_commits)
assert subprocess.run(['git','merge-base','--is-ancestor',remote_base,head],cwd=ROOT).returncode==0
assert branch == 'codex/mt-p1'
assert not git('status','--porcelain','--untracked-files=no').strip(), 'Tracked tree must be clean'
object_lines = git('rev-list', '--objects', *range_args).decode().splitlines()
object_names = {}
for line in object_lines:
    oid, _, path = line.partition(' ')
    object_names[oid] = path
objects = {}
raw = subprocess.run(['git', 'cat-file', '--batch'], cwd=ROOT,
                     input=('\n'.join(object_names) + '\n').encode(), capture_output=True, check=True).stdout
offset = 0
for expected_oid in object_names:
    end = raw.index(b'\n', offset)
    oid, kind, length = raw[offset:end].decode().split()
    size = int(length)
    data = raw[end + 1:end + 1 + size]
    assert oid == expected_oid and len(data) == size
    objects[oid] = {'kind': kind, 'bytes': size, 'data': data, 'path_hint': object_names[oid]}
    offset = end + 1 + size + 1
assert offset == len(raw)
blobs = {oid: v for oid, v in objects.items() if v['kind'] == 'blob'}


def tree(revision):
    result = {}
    for row in git('ls-tree', '-r', '-z', revision).split(b'\0'):
        if not row:
            continue
        meta, path = row.split(b'\t', 1)
        mode, kind, oid = meta.decode().split()
        result[path.decode()] = (mode, kind, oid)
    return result


base_tree = tree(remote_base)
head_tree = tree(head)
paths_by_oid = collections.defaultdict(set)
new_path_versions = set()
symlinks = {}
gitlinks = []
for commit in commits:
    current = tree(commit)
    for path, (mode, kind, oid) in current.items():
        if oid in blobs:
            paths_by_oid[oid].add(path)
        if base_tree.get(path) != (mode, kind, oid):
            new_path_versions.add((path, mode, kind, oid))
            if mode == '120000':
                payload = blobs[oid]['data'] if oid in blobs else git('cat-file', 'blob', oid)
                target = payload.decode('utf-8', 'replace')
                symlinks[(path, oid)] = {'path': path, 'oid': oid, 'bytes': len(payload),
                                        'absolute_target': target.startswith('/'),
                                        'parent_traversal': '..' in Path(target).parts,
                                        'target_sha256': sha(payload)}
            if mode == '160000':
                gitlinks.append({'path': path, 'oid': oid})


patterns = {
    'private_key_material_header': rb'-----BEGIN (?:RSA |EC |DSA |OPENSSH |PGP )?PRIVATE KEY-----',
    'github_token': rb'(?<![A-Za-z0-9])(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})',
    'openai_style_secret': rb'(?<![A-Za-z0-9])sk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{20,}',
    'aws_access_key_id': rb'(?<![A-Z0-9])(?:AKIA|ASIA)[A-Z0-9]{16}(?![A-Z0-9])',
    'google_api_key': rb'AIza[A-Za-z0-9_-]{35}',
    'google_oauth_secret': rb'GOCSPX-[A-Za-z0-9_-]{20,}',
    'huggingface_token': rb'(?<![A-Za-z0-9])hf_[A-Za-z0-9]{25,}',
    'slack_token': rb'xox[baprs]-[A-Za-z0-9-]{10,}',
    'jwt_like': rb'eyJ[A-Za-z0-9_-]{10,}\.eyJ[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}',
    'url_embedded_password': rb'https?://[^\s/:@"\']+:[^\s/@"\']+@',
    'credential_literal_assignment': rb'(?i)(?:api[_-]?key|access[_-]?token|auth[_-]?token|client[_-]?secret|secret[_-]?key|password|passwd)\s*["\']?\s*[:=]\s*["\']([^"\'\r\n]{6,})["\']',
}
compiled = {name: re.compile(expression) for name, expression in patterns.items()}
credential_hits = []
absolute_path_rx = re.compile(rb'/(?:home|Users|tmp|mnt|root|opt|usr|etc|var|data|workspace|workspaces)/[^\s"\'<>`\]\)\}]*')
sensitive_path_rx = re.compile(rb'/(?:\.ssh|\.aws|\.netrc|\.kube|\.gnupg)(?:/|$)|/(?:credentials|id_rsa|id_ed25519|authorized_keys|known_hosts|\.env)(?:[^A-Za-z0-9]|$)', re.I)
absolute = collections.defaultdict(lambda: {'occurrences': 0, 'oids': set(), 'paths': set()})
sensitive_paths = []
binary = []
path_samples = {}
placeholder_rx = re.compile(rb'(?i)^(?:none|null|false|true|redacted|not_recorded|not_implemented|example|dummy|placeholder|changeme|your[_ -]|test[_ -]|fake[_ -]|\*{3,}|<|\{|\$)')
for oid, obj in objects.items():
    if obj['kind'] not in ('blob', 'commit', 'tag'):
        continue
    data = obj['data']
    candidate_paths = sorted(paths_by_oid[oid]) if obj['kind'] == 'blob' else ['<commit-message-and-headers>']
    if obj['kind'] == 'blob':
        try:
            data.decode('utf-8')
            is_binary = b'\0' in data
        except UnicodeDecodeError:
            is_binary = True
        if is_binary:
            binary.append({'oid': oid, 'bytes': len(data), 'paths': candidate_paths})
    for name, expression in compiled.items():
        for match in expression.finditer(data):
            value = match.group(1) if name == 'credential_literal_assignment' else match.group(0)
            chars = collections.Counter(value)
            entropy = -sum((n / len(value)) * math.log2(n / len(value)) for n in chars.values())
            credential_hits.append({'pattern': name, 'oid': oid, 'kind': obj['kind'], 'paths': candidate_paths,
                                    'line': data.count(b'\n', 0, match.start()) + 1,
                                    'matched_bytes': len(value), 'entropy_bits_per_character': round(entropy, 3),
                                    'known_placeholder_prefix': bool(placeholder_rx.search(value)),
                                    'value_sha256': sha(value), 'classification': 'REVIEW_REQUIRED'})
    for match in absolute_path_rx.finditer(data):
        value = match.group(0)
        prefix_parts = value.decode('utf-8', 'replace').split('/')
        category = '/' + prefix_parts[1] + '/'
        if prefix_parts[1] in ('home', 'Users') and len(prefix_parts) > 2:
            category += '[USER]/'
        bucket = absolute[category]
        bucket['occurrences'] += 1
        bucket['oids'].add(oid)
        bucket['paths'].update(candidate_paths)
        if sensitive_path_rx.search(value):
            sensitive_paths.append({'oid': oid, 'paths': candidate_paths,
                                    'line': data.count(b'\n', 0, match.start()) + 1,
                                    'value_sha256': sha(value), 'category': category})

payload_ext = re.compile(r'(?i)\.(?:pth|pt|ckpt|safetensors|onnx|pkl|pickle|joblib|npy|npz|bin|bag|db|sqlite3?|zip|tar|tgz|gz|7z|mp4|avi|mov)$')
raw_names = {'episodes.jsonl', 'kroc2027_episodes.jsonl', 'occlusion_frames.bin', 'occlusion_layouts.npz',
             'recovery_events.jsonl', 'obs_dump.npz', 'trajectory_digest.jsonl'}
payload_paths = []
credential_paths = []
results_paths = set()
all_changed_paths = set()
for path, mode, kind, oid in sorted(new_path_versions):
    all_changed_paths.add(path)
    if path.startswith('results/'):
        results_paths.add(path)
    if payload_ext.search(path) or Path(path).name in raw_names:
        payload_paths.append({'path': path, 'mode': mode, 'oid': oid,
                              'bytes': objects.get(oid, {}).get('bytes'), 'classification': 'REVIEW_REQUIRED'})
    if re.search(r'(^|/)(?:\.env(?:\.[^/]+)?|\.netrc|id_rsa|id_ed25519|credentials|credential\.json|token\.json)$', path, re.I):
        credential_paths.append({'path': path, 'oid': oid})

# Current main worktree WIP is only read; neither its index nor stash is altered.
main_status = git('status', '--porcelain', '--untracked-files=no', root=MAIN).decode().splitlines()
wip = []
for row in main_status:
    path = row[3:]
    data = (MAIN / path).read_bytes()
    committed = git('show', 'HEAD:' + path, root=MAIN)
    base_lines = committed.decode('utf-8').splitlines(keepends=True)
    wip_lines = data.decode('utf-8').splitlines(keepends=True)
    blocks = []
    for op, i, j, k, l in difflib.SequenceMatcher(None, base_lines, wip_lines).get_opcodes():
        if op in ('insert', 'replace'):
            block = ''.join(wip_lines[k:l]).encode()
            matching = [oid for oid, obj in blobs.items() if block and block in obj['data']]
            blocks.append({'line_range': [k + 1, l], 'line_count': l - k,
                           'bytes': len(block), 'sha256': sha(block),
                           'matching_new_historical_blob_oids': matching})
    exact = [oid for oid, obj in blobs.items() if obj['data'] == data]
    wip.append({'path': path, 'status': row[:2], 'sha256': sha(data), 'bytes': len(data),
                'same_as_branch_head': git('show', head + ':' + path, check=False) == data,
                'exact_matches_any_new_historical_blob': exact, 'insert_replace_blocks': blocks,
                'main_head': git('rev-parse', 'HEAD', root=MAIN).decode().strip()})

head_symlinks = [(p, v) for p, v in head_tree.items() if v[0] == '120000']
old_head_symlinks = sum(base_tree.get(p) == value for p, value in head_symlinks)
report = {
    'schema': 'motar.research-branch.full-unpublished-history-audit.v1',
    'generated_at_utc': datetime.now(timezone.utc).isoformat(),
    'head': head, 'branch': branch, 'base_advertised_origin_research_branch': remote_base,
    'range': remote_base + '..' + head, 'commit_count': len(commits),
    'commits_oldest_first': commits, 'same_commit_set_as_HEAD_not_all_local_remote_refs': set(commits) == set(allremote_commits),
    'remote': {'configured': remote_urls, 'expected_target': EXPECTED_REMOTE,
               'read_only_ls_remote_return_code': remote.returncode, 'advertised': advertised,
               'destination_ref': 'refs/heads/codex/mt-p1', 'branch_absent_on_server_at_check': 'refs/heads/codex/mt-p1' not in advertised,
               'planned_ordinary_refspec_only': 'HEAD:refs/heads/codex/mt-p1', 'fetch_or_push_performed': False},
    'object_scan': {'unique_git_objects': len(objects), 'new_blob_count': len(blobs),
                    'new_blob_bytes': sum(v['bytes'] for v in blobs.values()),
                    'all_new_blob_bytes_and_commit_headers_messages_scanned': True,
                    'binary_blobs': binary, 'unscanned_binary_blob_count': len(binary),
                    'largest_new_blobs': sorted([{'oid': oid, 'bytes': v['bytes'], 'paths': sorted(paths_by_oid[oid])}
                                                  for oid, v in blobs.items()], key=lambda v: v['bytes'], reverse=True)[:8]},
    'secrets': {'scanner': 'independent finite byte-regex scan of every new blob and commit message; no size skips; quoted credential assignment candidates + entropy metadata',
                'patterns': list(patterns), 'candidates': credential_hits,
                'sensitive_credential_path_candidates': credential_paths,
                'external_scanner_availability': {n: bool(shutil.which(n)) for n in ('gitleaks', 'detect-secrets', 'trufflehog')},
                'credential_values_exported': False},
    'publication_payload': {'distinct_changed_paths': len(all_changed_paths),
                            'new_payload_extension_or_raw_name_candidates': payload_paths,
                            'results_distinct_changed_paths': sorted(results_paths),
                            'new_symlink_versions': list(symlinks.values()), 'new_gitlinks': gitlinks,
                            'head_total_symlink_paths': len(head_symlinks),
                            'head_symlinks_already_identical_on_advertised_origin_research_branch': old_head_symlinks,
                            'untracked_raw_symlinks': git('status', '--porcelain', '--untracked-files=normal').decode().splitlines()},
    'main_wip': wip,
    'local_path_disclosure': {'categories': {k: {'occurrences': v['occurrences'], 'unique_objects': len(v['oids']),
                                               'repository_paths': sorted(v['paths'])[:12]} for k, v in absolute.items()},
                              'sensitive_absolute_path_candidates': sensitive_paths,
                              'classification': 'REVIEW_REQUIRED until contextual review; absolute project/provenance paths are disclosed, not automatically secrets'},
    'limits': ['Finite pattern/assignment scan cannot prove absence of arbitrary encoded or unknown credentials.',
               'Already-advertised origin/codex/mt-p1 ancestry is not new publication; inherited symlinks are separately counted.',
               'No Git history rewritten or repository/index/remote ref modified.',
               'This audit binds the exact HEAD; any later commit or different push ref requires refreshed coverage.',
               'Remote advertisement can change after check; recheck before actual ordinary single-branch push.'],
    'overall': 'PENDING_MANUAL_CANDIDATE_CLASSIFICATION',
}
with OUT.with_suffix('.json').open('x') as stream:
    json.dump(report, stream, indent=2, sort_keys=True)
    stream.write('\n')
print(json.dumps({'head': head, 'commits': len(commits), 'blobs': len(blobs),
                  'bytes': sum(v['bytes'] for v in blobs.values()), 'binary': len(binary),
                  'credential_candidates': credential_hits, 'credential_path_candidates': credential_paths,
                  'payload_candidates': payload_paths, 'new_symlinks': list(symlinks.values()),
                  'new_gitlinks': gitlinks, 'main_wip': wip,
                  'sensitive_local_paths': sensitive_paths,
                  'local_path_categories': {k: v['occurrences'] for k, v in absolute.items()},
                  'head_symlinks': len(head_symlinks), 'already_remote_symlinks': old_head_symlinks,
                  'remote': report['remote']}, indent=2))
