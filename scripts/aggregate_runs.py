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
finish    : per-map goal (goal_x/goal_y/goal_radius) read from config/map_starts.yaml
            by the run's map token, or a global override via --finish-x/-y/-radius.
completed : reaching the finish region -- AFTER first leaving it by --arm-dist, so a
            lap whose finish == start is not "complete" at t=0 -- without collision.
            With no finish region, simply not colliding.
collision : first row whose d_min < --collision-radius (0.15 m). This is a terminal
            event: metrics are evaluated only up to it, so a crashed run's post-impact
            idle (car halted to run_timeout) never enters the averages. A finish
            reached only AFTER a collision does not count as completion.
t_course  : time at which the finish region is reached, reported for completed runs
            only. Metrics are evaluated up to the first terminal event (completion OR
            collision), so post-finish coasting, a second-lap crash, or post-impact
            idle does not pollute the reported course.
success rate : fraction of a config's seeds that completed.

Pure Python stdlib (csv, glob, statistics, argparse) so it runs anywhere.
"""
import argparse, csv, glob, math, os, re, statistics
from compute_ttc import min_ttc, min_rel_distance

FNAME_RE = re.compile(r"^(?P<config>.+)_map(?P<map>[^_]+)_seed(?P<seed>\d+)\.csv$")


def load_rows(path):
    # Tolerate a truncated / NUL-corrupted final line from a run killed mid-write:
    # strip NULs and let a short last row fall out (col() ignores missing fields).
    with open(path, newline="") as f:
        data = f.read().replace("\x00", "")
    return [{k: v for k, v in r.items()} for r in csv.DictReader(data.splitlines())]


def col(rows, name):
    out = []
    for r in rows:
        try:
            out.append(float(r[name]))
        except (KeyError, ValueError):
            pass
    return out


def parse_map_starts(path):
    """Minimal parser for config/map_starts.yaml -> {map_name: {key: float}}."""
    out, cur = {}, None
    if not path or not os.path.exists(path):
        return out
    with open(path) as f:
        for raw in f:
            line = raw.split("#", 1)[0].rstrip("\n")
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(" "))
            body = line.strip()
            if indent == 2 and body.endswith(":"):
                cur = body[:-1].strip()
                out[cur] = {}
            elif indent >= 4 and cur is not None and ":" in body:
                k, v = body.split(":", 1)
                try:
                    out[cur][k.strip()] = float(v.strip())
                except ValueError:
                    pass
    return out


def finish_for_map(map_token, map_goals, default_radius):
    """(goal_x, goal_y, radius) for a filename's map token, or None."""
    blk = map_goals.get("map" + str(map_token))
    if not blk or "goal_x" not in blk or "goal_y" not in blk:
        return None
    return (blk["goal_x"], blk["goal_y"], blk.get("goal_radius", default_radius))


def summarize_run(path, collision_radius, finish, arm_dist):
    rows = load_rows(path)
    if not rows:
        return None

    # collision: first row whose d_min falls below the collision radius. This is the
    # terminal event for a crashed run; metrics are evaluated only up to it (mirroring
    # the completion truncation below). Without this cap a run's post-impact idle
    # (car halted at v~0 with frozen geometry, right up to run_timeout) would be
    # folded into mean_v / mean_dmin / var_delta -- and by an amount that depends on
    # WHEN it crashed, which varies per config/seed. Truncating scores every config
    # over the same phase of motion (approach up to first sub-radius contact), so the
    # averages are consistent regardless of whether the simulator halts on collision.
    coll_idx = None
    for i, r in enumerate(rows):
        try:
            if float(r["d_min"]) < collision_radius:
                coll_idx = i
                break
        except (KeyError, ValueError):
            continue

    # completion: first row within the finish region AFTER first leaving it by
    # arm_dist, so a lap whose finish == start is not "complete" at t=0. A finish
    # reached only AFTER a collision does not count -- the crash terminates the run
    # first -- so the scan stops at the collision instant.
    reached, comp_idx = False, len(rows) - 1
    if finish is not None:
        fx, fy, fr = finish
        armed = False
        for i, r in enumerate(rows):
            if coll_idx is not None and i > coll_idx:
                break
            try:
                d = math.hypot(float(r["x"]) - fx, float(r["y"]) - fy)
            except (KeyError, ValueError):
                continue
            if d > arm_dist:
                armed = True
            if armed and d < fr:
                reached, comp_idx = True, i
                break

    # Evaluate metrics only up to whichever terminal event comes first -- completion
    # or collision -- so post-finish coasting, a crash on a second lap, or post-impact
    # idle does not pollute the reported course.
    end_idx = comp_idx if coll_idx is None else min(comp_idx, coll_idx)
    rows = rows[:end_idx + 1]

    dmin = col(rows, "d_min")
    v = col(rows, "v_cmd")
    dsteer = [abs(x) for x in col(rows, "delta_cmd")]
    iters = col(rows, "iters")
    timeout = col(rows, "timeout")
    Ji, Jf = col(rows, "J_init"), col(rows, "J_final")
    t = col(rows, "t")

    min_dmin = min(dmin) if dmin else math.nan
    collided = (min_dmin < collision_radius) if dmin else True

    # completion / course time (t_course reported for completed runs only)
    if finish is not None:
        completed = reached and not collided
        t_course = (t[-1] if t else math.nan) if reached else math.nan
    else:
        completed = not collided
        t_course = t[-1] if t else math.nan

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
    ap.add_argument("--finish-radius", type=float, default=1.0)
    ap.add_argument("--arm-dist", type=float, default=2.0,
                    help="ego must leave the finish region by this much before a "
                         "return counts as completion (matches goal_arm_dist)")
    ap.add_argument("--map-starts",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "config", "map_starts.yaml"),
                    help="per-map goals; used when --finish-x/-y are not given")
    ap.add_argument("--out", default="summary.csv")
    ap.add_argument("--latex", action="store_true",
                    help="also print LaTeX \\ms{mean}{std} cells per config")
    args = ap.parse_args()

    # A global --finish-x/-y overrides everything; otherwise each run's finish is
    # looked up per map from map_starts.yaml (single source of truth).
    global_finish = None
    if args.finish_x is not None and args.finish_y is not None:
        global_finish = (args.finish_x, args.finish_y, args.finish_radius)
    map_goals = parse_map_starts(args.map_starts)

    runs = {}  # config -> list of per-run dicts
    for path in sorted(glob.glob(os.path.join(args.log_dir, "*.csv"))):
        m = FNAME_RE.match(os.path.basename(path))
        if not m:
            print("  skip (name):", os.path.basename(path))
            continue
        finish = global_finish if global_finish is not None \
            else finish_for_map(m.group("map"), map_goals, args.finish_radius)
        s = summarize_run(path, args.collision_radius, finish, args.arm_dist)
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
