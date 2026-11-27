"""Read-only audit of every commit reachable from HEAD but not from any remote-tracking ref."""
import json, re, subprocess, sys, collections
def git(*a, **k): return subprocess.run(['git', *a], capture_output=True, check=True, **k).stdout
commits = git('rev-list', 'HEAD', '--not', '--remotes', text=True).split()
# Every new object (with first path) that the push would transfer.
objs = [l.split(' ', 1) for l in git('rev-list', '--objects', 'HEAD', '--not', '--remotes', text=True).splitlines()]
batch = git('cat-file', '--batch-check=%(objectname) %(objecttype) %(objectsize)', input='\n'.join(o[0] for o in objs).encode()).decode().split('\n')
info = {l.split()[0]: (l.split()[1], int(l.split()[2])) for l in batch if l.strip()}
blobs = [(o[0], o[1] if len(o) > 1 else '', info[o[0]][1]) for o in objs if info[o[0]][0] == 'blob']
RAW_EXT = re.compile(r'\.(pth|pt|ckpt|onnx|npz|npy|bin|h5|hdf5|pkl|pickle|zip|tar|gz|tgz|7z|mp4|avi|bag|db3|safetensors)$', re.I)
raw_like = [(p, s) for _, p, s in blobs if RAW_EXT.search(p) or p.startswith('results/') or '/results/' in p or p.startswith('runs/') or '/nn/' in p]
large = sorted([(p, s) for _, p, s in blobs if s > 2_000_000], key=lambda x: -x[1])
# Symlinks and submodules in new trees, per commit.
special = set()
for c in commits:
    for line in git('ls-tree', '-r', c, text=True).splitlines():
        mode, typ, sha, path = line.split(None, 3)
        if mode in ('120000', '160000') and sha in info: special.add((mode, path))
# Added lines across each unpublished commit (first parent diff; root-safe).
SECRET = re.compile(r'(-----BEGIN [A-Z ]*PRIVATE KEY-----|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|gho_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|xox[bpas]-[A-Za-z0-9-]{10,}|sk-[A-Za-z0-9]{20,}|AIza[0-9A-Za-z_-]{30,}|(?i:(api[_-]?key|secret[_-]?key|access[_-]?token|password|passwd)\s*[:=]\s*["\'][^"\']{6,}))')
LOCAL = re.compile(r'(/home/[A-Za-z0-9_.-]+|/tmp/[A-Za-z0-9_.-]+|/Users/[A-Za-z0-9_.-]+|C:\\\\Users)')
secrets, local_paths, local_by_file = [], collections.Counter(), collections.Counter()
for c in commits:
    diff = git('show', '--format=', '--unified=0', '--no-color', '--no-renames', c, text=True, errors='replace')
    path = None
    for line in diff.splitlines():
        if line.startswith('+++ '): path = line[6:] if line.startswith('+++ b/') else line[4:]; continue
        if line.startswith('+') and not line.startswith('+++'):
            if SECRET.search(line): secrets.append((c[:9], path, line[:160]))
            for m in LOCAL.findall(line):
                local_paths[m] += 1; local_by_file[path] += 1
sensitive_names = [p for _, p, _ in blobs if re.search(r'(^|/)(\.env(\..*)?|id_rsa|id_ed25519|.*\.pem|.*\.key|credentials(\.json)?|\.netrc|\.pypirc)$', p)]
# User WIP in the main checkout and its stash must not be in the pushed history.
main = '/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator'
wip = {}
for rel in ('.gitignore', 'WORKLOG.md', 'presentations/README.md'):
    h = git('hash-object', main + '/' + rel, text=True).strip(); wip[rel] = {'blob': h, 'in_push': h in info}
stash = '23ddb33f92cf6c75cf9095e149bb7e2516eb7543'
stash_reachable = subprocess.run(['git', 'merge-base', '--is-ancestor', stash, 'HEAD']).returncode == 0
report = {'commits': len(commits), 'new_objects': len(objs), 'new_blobs': len(blobs),
          'new_blob_bytes': sum(s for *_, s in blobs), 'raw_like_paths': raw_like, 'blobs_over_2MB': large,
          'symlinks_or_submodules': sorted(special), 'secret_pattern_hits': secrets, 'sensitive_filenames': sensitive_names,
          'local_path_tokens': local_paths.most_common(), 'local_path_files_top': local_by_file.most_common(25),
          'local_path_files_count': len(local_by_file), 'main_wip_blobs': wip, 'stash_23ddb33_ancestor_of_HEAD': stash_reachable}
json.dump(report, open(sys.argv[1], 'w'), indent=1); print(json.dumps({k: (v if not isinstance(v, list) or len(v) < 30 else v[:30]) for k, v in report.items()}, indent=1)[:9000])
