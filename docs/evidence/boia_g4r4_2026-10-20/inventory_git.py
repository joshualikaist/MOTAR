"""Metadata-only NUL-safe Git inventory; no symlink following or Raw reads."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess


def parse_status(data):
    parts = data.split(b"\0")
    rows, i = [], 0
    while i < len(parts) - 1:
        item = parts[i]
        i += 1
        status, path = item[:2].decode("ascii"), os.fsdecode(item[3:])
        origin = None
        if "R" in status or "C" in status:
            origin = os.fsdecode(parts[i])
            i += 1
        rows.append(dict(status=status, path=path, rename_source=origin))
    return rows


def inventory(root, include_ignored=False):
    rows = parse_status(subprocess.check_output([
        "git", "-C", str(root), "status", "--porcelain=v1", "-z", "--untracked-files=all"]))
    for row in rows:
        p = root / row["path"]
        if p.exists() or p.is_symlink():
            st = p.lstat()
            row.update(logical_bytes=st.st_size, allocated_bytes=st.st_blocks * 512,
                       symlink=p.is_symlink(), link_target=os.readlink(p) if p.is_symlink() else None)
        else:
            row.update(logical_bytes=0, allocated_bytes=0, deleted=True)
    staged = {x["path"] for x in rows if x["status"][0] not in " ?!"}
    unstaged = {x["path"] for x in rows if x["status"][1] not in " ?!"}
    counts = dict(staged=len(staged), unstaged=len(unstaged),
                  untracked=sum(x["status"] == "??" for x in rows),
                  unique=len({x["path"] for x in rows}), overlap=len(staged & unstaged))
    ignored = None
    if include_ignored:
        raw = subprocess.check_output(["git", "-C", str(root), "ls-files", "--others",
                                       "--ignored", "--exclude-standard", "-z"])
        ignored = [os.fsdecode(x) for x in raw.split(b"\0") if x]
        counts["ignored"] = len(ignored)
    return dict(root=str(root), counts=counts, records=rows, ignored_paths=ignored,
                top20=Counter(str(Path(x["path"]).parent) for x in rows).most_common(20),
                extensions=dict(Counter(Path(x["path"]).suffix or "<none>" for x in rows)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--output", required=True)
    parser.add_argument("--ignored", action="store_true")
    args = parser.parse_args()
    # Literal embedded newline/tab/spaces and a rename source cannot become rows.
    probe = parse_status(b"?? a b\nname\t.py\0RM renamed\nfile\0old name\n.py\0")
    assert len(probe) == 2 and probe[0]["path"] == "a b\nname\t.py"
    assert probe[1]["rename_source"] == "old name\n.py"
    result = inventory(args.root, args.ignored)
    result["parser_control_character_selftest"] = "PASS"
    with open(args.output, "x") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result["counts"]))


if __name__ == "__main__":
    main()
