# Research-branch publication checks

Authorization is ordinary fast-forward push of `codex/mt-p1` only, after the user's history,
security, WIP/raw/checkpoint/symlink/local-path and remote-target checks. Main is not a destination.

An initial read-only audit of **every unpublished commit/object** from advertised branch base
`aed6affbd2567feee79e658fbe5f07510b039f36` through result commit `d063e7d` found:

- 8 commits, 41 new blobs, 13,937,270 blob bytes; no binary blob skipped;
- no secret-pattern/quoted-credential candidate, credential path, raw/checkpoint payload,
  new symlink or gitlink;
- all 296 tracked symlinks already identical at the advertised research-branch base;
- no exact user-WIP blob or inserted/replaced WIP block in any unpublished historical blob;
- project interpreter/import/provenance/scratch/system executable paths are disclosed in
  compact records and code. They contain no detected credential path or credential value.
  `/root/` and `/opt/` matches also include inherited WORKLOG text and source path-expression
  fragments; they are not new credential material. Existing published path disclosures are
  not silently removed by rewriting history.

This initial audit does **not** certify later review/report commits. Before the actual push,
rerun the same full byte scan on the exact final HEAD, including every new commit header/message
and all versions of new blobs, with no size skip. Confirm no tracked dirt; explicit owned paths
only; baseline main/WIP/stash/raw/checkpoint hashes unchanged; exact fetch/push remote
`git@github.com:joshualikaist/MOTAR.git`; branch ancestry is fast-forward from the freshly
advertised research-branch SHA. Re-advertise immediately before pushing the single explicit ref:

```bash
git push origin HEAD:refs/heads/codex/mt-p1
git ls-remote --heads origin refs/heads/codex/mt-p1 refs/heads/main
```

No force option, main refspec, merge, history rewrite, raw staging or WIP/stash operation is
permitted. A changed remote or unexpected scan candidate blocks publication until understood.
The finite pattern scan cannot prove absence of arbitrary encoded credentials. Exact final
audit/advertisement/push outputs are retained under `/tmp/motar_s_l1c_v2_20261018/` and the actual
remote SHA is reported to the user; no push success is implied at this document's commit time.
