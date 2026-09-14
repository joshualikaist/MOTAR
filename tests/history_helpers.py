"""Skip, rather than error, when a check needs research history this checkout does not have.

Several evidence tests verify a claim against a specific commit of the research repository: that a
historical blob is unchanged, that a receipt matches the tree it recorded. A public release snapshot
deliberately carries content without history, so those commits are absent there. The check is not
failing in that case - it cannot run, and saying so is different from saying the evidence is wrong.

Inside the research repository the same guard is inert: the commits exist and the checks run.
"""
import subprocess
import unittest


def git_output(arguments, cwd):
    try:
        return subprocess.check_output(["git", *arguments], cwd=str(cwd), text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def inside_git_work_tree(root):
    return git_output(["rev-parse", "--is-inside-work-tree"], root) == "true"


def has_commit(root, commit):
    return git_output(["cat-file", "-t", commit], root) == "commit"


def require_research_history(root, *commits):
    """Raise SkipTest when this checkout cannot see the history the check is about."""
    if not inside_git_work_tree(root):
        raise unittest.SkipTest(
            "not inside a Git work tree: this check verifies research history, which a release "
            "snapshot does not carry")
    missing = [commit for commit in commits if not has_commit(root, commit)]
    if missing:
        raise unittest.SkipTest(
            "research history not present in this checkout (missing %s); a release snapshot carries "
            "content without history" % ", ".join(sorted(missing)[:3]))
