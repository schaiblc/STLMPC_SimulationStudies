"""aggregate_runs.py -- turn per-run telemetry CSVs into the paper's tables.

Reads a directory of per-control-step CSVs written by the RunLogger (both STLMPC
nodes and the FGM node), one file per run, named:

    <CONFIG>_map<M>_seed<NN>.csv        e.g.  B2_V2_map3_seed03.csv

CONFIG is any label without spaces; map<M> and seed<NN> are required tokens.
For each run it computes the summary metrics used in the manuscript, then
aggregates across seeds within a CONFIG to mean +/- std and a success rate.
Output: a tidy summary CSV, and (optionally) ready-to-paste LaTeX \ms{}{} cells
for Table \ref{tab:sim_ablation} / \ref{tab:sim_robust}.

Definitions
-----------
collision : min over the run of d_min < --collision-radius (default 0.15 m).
completed : if a finish region is given (--finish-x/-y/-radius), reaching it
            without collision; otherwise, simply not colliding.
t_course  : time to reach the finish region (if given) else the run duration,
            reported for completed runs only.
success rate : fraction of a config's seeds that completed.

Pure Python stdlib (csv, glob, statistics, argparse) so it runs anywhere.
"""
import argparse, csv, glob, math, os, re, statistics
from compute_ttc import min_ttc, min_rel_distance

FNAME_RE = re.compile(r"^(?P<config>.+)_map(?P<map>[^_]+)_seed(?P<seed>\d+)\.csv$")


def load_rows(path):
    with open(path, newline="") as f:
        return [{k: v for k, v in r.items()} for r in csv.DictReader(f)]


def col(rows, name):
    out = []
    for r in rows:
        try:
            out.append(float(r[name]))
        except (KeyError, ValueError):
            pass
    return out


def summarize_run(path, collision_radius, finish):
    rows = load_rows(path)
    if not rows:
        return None
    dmin = col(rows, "d_min")
    v = col(rows, "v_cmd")
    dsteer = [abs(x) for x in col(rows, "delta_cmd")]
    iters = col(rows, "iters")
    timeout = col(rows, "timeout")
    Ji, Jf = col(rows, "J_init"), col(rows, "J_final")
    t = col(rows, "t")

    min_dmin = min(dmin) if dmin else math.nan
    collided = (min_dmin < collision_radius) if dmin else True

    # completion / course time
    reached, t_course = False, (t[-1] if t else math.nan)
    if finish is not None:
        fx, fy, fr = finish
        for r in rows:
            try:
                if math.hypot(float(r["x"]) - fx, float(r["y"]) - fy) < fr:
                    reached, t_course = True, float(r["t"])
                    break
            except (KeyError, ValueError):
                pass
        completed = reached and not collided
    else:
        completed = not collided

    def mean(a):
        return statistics.fmean(a) if a else math.nan

    def var(a):
        return statistics.pvariance(a) if len(a) > 1 else 0.0

    jratio = math.nan
    if Ji and Jf and mean(Ji) not in (0.0, math.nan):
        jratio = mean(Jf) / mean(Ji)

    return {
        "completed": 1.0 if completed else 0.0,
        "collided": 1.0 if collided else 0.0,
        "t_course": t_course if completed else math.nan,
        "min_dmin": min_dmin,
        "mean_dmin": mean(dmin),
        "mean_abs_delta": mean(dsteer),
        "var_delta": var(col(rows, "delta_cmd")),
        "mean_v": mean(v),
        "var_v": var(v),
        "mean_iters": mean(iters),
        "max_iters": max(iters) if iters else math.nan,
        "timeout_frac": mean(timeout) if timeout else 0.0,
        "j_ratio": jratio,
        "min_ttc": min_ttc(rows),
        "min_rel_dist": min_rel_distance(rows),
    }


def agg(vals):
    vals = [x for x in vals if isinstance(x, float) and math.isfinite(x)]
    if not vals:
        return (math.nan, math.nan)
    m = statistics.fmean(vals)
    s = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return (m, s)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log_dir", help="directory of *_map<M>_seed<NN>.csv run files")
    ap.add_argument("--collision-radius", type=float, default=0.15)
    ap.add_argument("--finish-x", type=float)
    ap.add_argument("--finish-y", type=float)
    ap.add_argument("--finish-radius", type=float, default=0.5)
    ap.add_argument("--out", default="summary.csv")
    ap.add_argument("--latex", action="store_true",
                    help="also print LaTeX \\ms{mean}{std} cells per config")
    args = ap.parse_args()

    finish = None
    if args.finish_x is not None and args.finish_y is not None:
        finish = (args.finish_x, args.finish_y, args.finish_radius)

    runs = {}  # config -> list of per-run dicts
    for path in sorted(glob.glob(os.path.join(args.log_dir, "*.csv"))):
        m = FNAME_RE.match(os.path.basename(path))
        if not m:
            print("  skip (name):", os.path.basename(path))
            continue
        s = summarize_run(path, args.collision_radius, finish)
        if s:
            runs.setdefault(m.group("config"), []).append(s)

    metrics = ["min_dmin", "mean_dmin", "mean_abs_delta", "var_delta", "mean_v",
               "var_v", "t_course", "mean_iters", "max_iters", "timeout_frac",
               "j_ratio", "min_ttc"]

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["config", "n_seeds", "success_rate"] +
                   [x + "_mean" for x in metrics] + [x + "_std" for x in metrics])
        for cfg in sorted(runs):
            rs = runs[cfg]
            n = len(rs)
            succ = statistics.fmean([r["completed"] for r in rs]) if rs else 0.0
            means, stds = [], []
            for mkey in metrics:
                mu, sd = agg([r[mkey] for r in rs])
                means.append(mu); stds.append(sd)
            w.writerow([cfg, n, "%.3f" % succ] +
                       ["%.4f" % x for x in means] + ["%.4f" % x for x in stds])
            if args.latex:
                def cell(mkey, prec=3):
                    mu, sd = agg([r[mkey] for r in rs])
                    if not math.isfinite(mu):
                        return "---"
                    return "\\ms{%.*f}{%.*f}" % (prec, mu, prec, sd)
                print("%% %s (n=%d)" % (cfg, n))
                print("  & %.0f & %s & %s & %s & %s \\\\" % (
                    100 * succ, cell("min_dmin"), cell("mean_dmin"),
                    cell("var_delta"), cell("mean_v")))
    print("wrote", args.out, "(%d configs)" % len(runs))


if __name__ == "__main__":
    main()
