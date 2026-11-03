/* Exact C port of cplanner/src/coverage_b.py (`point_segment`, `crosses`, `segment_box_distance`, `_coverage_B`).
 *
 * M3 candidate, NOT used by any qualified configuration until its equivalence gate passes. The port keeps the
 * verbatim traversal (depth-first, same push order, same node counting) and the same IEEE-754 double arithmetic in the
 * same evaluation order, so the whole result (certified, reason, nodes, shape, cell) is meant to be identical:
 *  - Python's max/min/sorted on two floats are reproduced with their exact tie rules (first argument kept unless the
 *    second is strictly greater / smaller);
 *  - math.hypot is CPython 3.8's two-argument `vector_norm` (scaled, compensated sum, one sqrt);
 *  - build flags forbid contraction and fast-math (build.py: -O2 -ffp-contract=off -fno-fast-math, SSE2 doubles).
 * Bounded memory: the depth-first stack holds at most one pending sibling per level (depth <= 24), so STACK_MAX 64
 * is never reached; overflow returns an error code that the wrapper turns into a refusal.
 */
#include <math.h>

#define STACK_MAX 64

enum { R_CERTIFIED = 0, R_SURFACE = 1, R_NO_RAYS = 2, R_BUDGET = 3, R_UNRESOLVED = 4, R_STACK = -1 };

static double py_max2(double a, double b) { return (b > a) ? b : a; }   /* max(a, b): a unless b > a */
static double py_min2(double a, double b) { return (b < a) ? b : a; }   /* min(a, b): a unless b < a */

/* CPython 3.8 Modules/mathmodule.c: math_hypot -> vector_norm(n=2) */
static double py_hypot(double x, double y)
{
    double v[2], mx = 0.0, csum = 1.0, oldcsum, frac = 0.0, t;
    int i, nan = 0;
    v[0] = fabs(x);
    v[1] = fabs(y);
    for (i = 0; i < 2; i++) {
        nan |= isnan(v[i]);
        if (v[i] > mx) mx = v[i];
    }
    if (isinf(mx)) return mx;
    if (nan) return NAN;
    if (mx == 0.0) return mx;
    for (i = 0; i < 2; i++) {
        t = v[i] / mx;
        t = t * t;
        oldcsum = csum;
        csum += t;
        frac += (oldcsum - csum) + t;
    }
    return mx * sqrt(csum - 1.0 + frac);
}

static double point_segment(double px, double py, double ax, double ay, double bx, double by)
{
    double dx = bx - ax, dy = by - ay, den = dx * dx + dy * dy, t;
    if (den != 0.0)
        t = py_max2(0., py_min2(1., ((px - ax) * dx + (py - ay) * dy) / den));
    else
        t = 0.;
    return py_hypot(px - ax - t * dx, py - ay - t * dy);
}

static int crosses(double ax, double ay, double bx, double by,
                   double xmin, double ymin, double xmax, double ymax, int strict)
{
    double lo = 0., hi = 1., x, d, mn, mx, u, w, t0, t1, t, X, Y;
    int k;
    for (k = 0; k < 2; k++) {
        if (k == 0) { x = ax; d = bx - ax; mn = xmin; mx = xmax; }
        else        { x = ay; d = by - ay; mn = ymin; mx = ymax; }
        if (fabs(d) < 1e-15) {
            if (x < mn || x > mx) return 0;
        } else {
            u = (mn - x) / d;
            w = (mx - x) / d;
            if (w < u) { t0 = w; t1 = u; } else { t0 = u; t1 = w; }   /* sorted((u, w)) */
            lo = py_max2(lo, t0);
            hi = py_min2(hi, t1);
            if (lo > hi) return 0;
        }
    }
    if (!strict) return 1;
    t = (lo + hi) / 2;
    X = ax + t * (bx - ax);
    Y = ay + t * (by - ay);
    return (xmin + 1e-9 < X && X < xmax - 1e-9 && ymin + 1e-9 < Y && Y < ymax - 1e-9);
}

static double segment_box_distance(double ax, double ay, double bx, double by,
                                   double x0, double y0, double x1, double y1)
{
    double v, best, gx, gy;
    if (crosses(ax, ay, bx, by, x0, y0, x1, y1, 0)) return 0.;
    best = point_segment(x0, y0, ax, ay, bx, by);
    v = point_segment(x0, y1, ax, ay, bx, by); best = py_min2(best, v);
    v = point_segment(x1, y0, ax, ay, bx, by); best = py_min2(best, v);
    v = point_segment(x1, y1, ax, ay, bx, by); best = py_min2(best, v);
    /* math.hypot(max(x0 - p[0], 0, p[0] - x1), max(y0 - p[1], 0, p[1] - y1)) for p in (a, b) */
    gx = py_max2(py_max2(x0 - ax, 0.0), ax - x1);
    gy = py_max2(py_max2(y0 - ay, 0.0), ay - y1);
    v = py_hypot(gx, gy); best = py_min2(best, v);
    gx = py_max2(py_max2(x0 - bx, 0.0), bx - x1);
    gy = py_max2(py_max2(y0 - by, 0.0), by - y1);
    v = py_hypot(gx, gy); best = py_min2(best, v);
    return best;
}

/* free: n_free x 4 (sx, sy, ex, ey); surf: n_surf x 2; shapes: n_shapes x 2 (hx, hy).
 * out_nodes: total nodes; out_shape[2], out_cell[4] for UNRESOLVED. Returns a reason code. */
int coverage_b(double ax, double ay, double bx, double by,
               const double *free, long n_free, const double *surf, long n_surf,
               const double *shapes, long n_shapes, double radius, double resolution, long max_nodes,
               long *out_nodes, double *out_shape, double *out_cell)
{
    double st[STACK_MAX][4];
    int sd[STACK_MAX];
    long total = 0, nodes, i, s, f;
    int top, dep, covered;
    double hx, hy, reach, x0, y0, x1, y1, cx, cy, ex, ey;

    *out_nodes = 0;
    for (i = 0; i < n_surf; i++)
        if (point_segment(surf[2 * i], surf[2 * i + 1], ax, ay, bx, by) <= radius + 1e-9) return R_SURFACE;
    if (n_free == 0) return R_NO_RAYS;
    for (s = 0; s < n_shapes; s++) {
        hx = shapes[2 * s];
        hy = shapes[2 * s + 1];
        reach = radius;
        top = 0;
        st[0][0] = py_min2(ax, bx) - reach - hx;
        st[0][1] = py_min2(ay, by) - reach - hy;
        st[0][2] = py_max2(ax, bx) + reach + hx;
        st[0][3] = py_max2(ay, by) + reach + hy;
        sd[0] = 0;
        top = 1;
        nodes = 0;
        while (top > 0) {
            top--;
            x0 = st[top][0]; y0 = st[top][1]; x1 = st[top][2]; y1 = st[top][3]; dep = sd[top];
            nodes++; total++;
            if (nodes > max_nodes) { *out_nodes = total; return R_BUDGET; }
            if (segment_box_distance(ax, ay, bx, by, x0 - hx, y0 - hy, x1 + hx, y1 + hy) > reach + 1e-9) continue;
            cx = (x0 + x1) / 2; cy = (y0 + y1) / 2; ex = (x1 - x0) / 2; ey = (y1 - y0) / 2;
            covered = 0;
            if (hx > ex && hy > ey) {
                for (f = 0; f < n_free; f++) {
                    if (crosses(free[4 * f], free[4 * f + 1], free[4 * f + 2], free[4 * f + 3],
                                cx - hx + ex, cy - hy + ey, cx + hx - ex, cy + hy - ey, 1)) {
                        covered = 1;
                        break;
                    }
                }
            }
            if (covered) continue;
            if (py_max2(x1 - x0, y1 - y0) <= resolution || dep >= 24) {
                *out_nodes = total;
                out_shape[0] = hx; out_shape[1] = hy;
                out_cell[0] = x0; out_cell[1] = y0; out_cell[2] = x1; out_cell[3] = y1;
                return R_UNRESOLVED;
            }
            if (top + 2 > STACK_MAX) { *out_nodes = total; return R_STACK; }
            if (ex >= ey) {
                st[top][0] = x0; st[top][1] = y0; st[top][2] = cx; st[top][3] = y1; sd[top] = dep + 1; top++;
                st[top][0] = cx; st[top][1] = y0; st[top][2] = x1; st[top][3] = y1; sd[top] = dep + 1; top++;
            } else {
                st[top][0] = x0; st[top][1] = y0; st[top][2] = x1; st[top][3] = cy; sd[top] = dep + 1; top++;
                st[top][0] = x0; st[top][1] = cy; st[top][2] = x1; st[top][3] = y1; sd[top] = dep + 1; top++;
            }
        }
    }
    *out_nodes = total;
    return R_CERTIFIED;
}

/* exported for the bit-identity test of the hypot reproduction */
double cov_py_hypot(double x, double y) { return py_hypot(x, y); }
double cov_point_segment(double px, double py, double ax, double ay, double bx, double by)
{
    return point_segment(px, py, ax, ay, bx, by);
}
double cov_segment_box_distance(double ax, double ay, double bx, double by, double x0, double y0, double x1, double y1)
{
    return segment_box_distance(ax, ay, bx, by, x0, y0, x1, y1);
}
int cov_crosses(double ax, double ay, double bx, double by, double x0, double y0, double x1, double y1, int strict)
{
    return crosses(ax, ay, bx, by, x0, y0, x1, y1, strict);
}
