#!/usr/bin/env python3
"""Binomial contrasts and pooling for the governor experiments, defined once.

Every governor number in this project is a rate over independent episodes, every claim is a
difference of two such rates, and every headline is those differences pooled over densities and
evaluation seeds. That arithmetic had been retyped in each summary script, which is how a progress
view and a final table start disagreeing about the same cell. This module is the single definition.

Counts are the input, never rates: `docs/plans/confirmation_phase_plan_2026-09-06.md` §4 freezes
"every table carries counts/n and derives the rate from them".

CPU-only, standard library only, no simulator import.
"""

from __future__ import annotations

import math

Z95 = 1.959963984540054

# outcome rate field -> the count field the evaluator writes beside it
COUNT_FIELD = {"crash_rate": "crash", "capture_rate": "captured", "timeout_rate": "timeout"}


def outcome_count(outcome, field):
    """The integer count behind an outcome rate, preferring the recorded count over rate x n."""
    name = COUNT_FIELD.get(field)
    if name is not None and name in outcome:
        return int(outcome[name])
    raise KeyError(f"no count recorded for {field}; outcome has {sorted(outcome)}")


def wald_diff(a_count, a_total, b_count, b_total, scale=100.0):
    """(delta, se) of pa - pb on the given scale (default percentage points).

    Wald, i.e. the normal approximation with each arm's own variance. Both arms are independent
    evaluations, so the variances add.
    """
    if a_total <= 0 or b_total <= 0:
        raise ValueError("empty arm")
    pa, pb = a_count / a_total, b_count / b_total
    delta = scale * (pa - pb)
    se = scale * math.sqrt(pa * (1.0 - pa) / a_total + pb * (1.0 - pb) / b_total)
    return delta, se


def ci(delta, se, z=Z95):
    return delta - z * se, delta + z * se


def excludes_zero(delta, se, z=Z95):
    lo, hi = ci(delta, se, z)
    return lo > 0.0 or hi < 0.0


def two_sided_p(delta, se):
    if se <= 0.0:
        return 0.0 if delta else 1.0
    return math.erfc(abs(delta) / (se * math.sqrt(2.0)))


def pool_fixed(estimates):
    """Inverse-variance fixed-effect pool of (delta, se) pairs -> (delta, se)."""
    estimates = list(estimates)
    if not estimates:
        raise ValueError("nothing to pool")
    weights = [1.0 / se ** 2 for _, se in estimates]
    total = sum(weights)
    delta = sum(w * d for w, (d, _) in zip(weights, estimates)) / total
    return delta, math.sqrt(1.0 / total)


def cochran_q(estimates):
    """(Q, df, I^2) heterogeneity of (delta, se) pairs. I^2 is clamped at 0."""
    estimates = list(estimates)
    centre, _ = pool_fixed(estimates)
    q = sum((d - centre) ** 2 / se ** 2 for d, se in estimates)
    df = len(estimates) - 1
    i2 = max(0.0, (q - df) / q) if q > 0.0 else 0.0
    return q, df, i2


def pool_random(estimates):
    """DerSimonian-Laird random-effect pool -> (delta, se, tau^2).

    Reported next to the fixed-effect pool whenever cells differ by construction (densities are not
    replicates of one another), so a reader can see how much the interval widens once between-cell
    variance is paid for.
    """
    estimates = list(estimates)
    q, df, _ = cochran_q(estimates)
    weights = [1.0 / se ** 2 for _, se in estimates]
    total = sum(weights)
    c = total - sum(w * w for w in weights) / total
    tau2 = max(0.0, (q - df) / c) if c > 0.0 else 0.0
    rw = [1.0 / (se ** 2 + tau2) for _, se in estimates]
    rtotal = sum(rw)
    delta = sum(w * d for w, (d, _) in zip(rw, estimates)) / rtotal
    return delta, math.sqrt(1.0 / rtotal), tau2


def holm(pvalues):
    """Holm-Bonferroni adjusted p-values, in the order given."""
    order = sorted(range(len(pvalues)), key=lambda i: pvalues[i])
    adjusted = [0.0] * len(pvalues)
    running = 0.0
    for rank, index in enumerate(order):
        value = (len(pvalues) - rank) * pvalues[index]
        running = max(running, min(1.0, value))
        adjusted[index] = running
    return adjusted


def sign_test_p(successes, trials):
    """Two-sided exact sign test against p = 1/2."""
    if trials <= 0:
        raise ValueError("no trials")
    k = min(successes, trials - successes)
    tail = sum(math.comb(trials, i) for i in range(0, k + 1)) / (2.0 ** trials)
    return min(1.0, 2.0 * tail)


def format_ci(delta, se, digits=2, z=Z95):
    lo, hi = ci(delta, se, z)
    return f"{delta:+.{digits}f} [{lo:+.{digits}f}, {hi:+.{digits}f}]"
