#!/usr/bin/env python3
"""check_run.py -- quick PASS/FAIL status for run CSVs (use while a sweep runs).

Reuses aggregate_runs.py's per-map goal + completion logic, so a verdict here
matches what the final aggregation will report. Point it at a directory (default
~/stlmpc_logs) or a single CSV.

Verdict per run:
  PASS        reached the goal (armed) without collision      -> shows t_course
  COLLISION   d_min dropped below --collision-radius
  INCOMPLETE  goal not reached yet (still running, or timed out)
  EMPTY       no rows yet (0-byte / just created)

Live monitor:  watch -n 5 python3 scripts/check_run.py
"""
import argparse, glob, math, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aggregate_runs import (FNAME_RE, load_rows, parse_map_starts,
                            finish_for_map, summarize_run, warn_arm_dist)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", default=os.path.expanduser("~/stlmpc_logs"),
                    help="log directory or a single CSV (default ~/stlmpc_logs)")
    ap.add_argument("--collision-radius", type=float, default=0.15)
    ap.add_argument("--arm-dist", type=float, default=5.0,
                    help="must match goal_arm_dist in campaign.launch (default 5.0)")
    ap.add_argument("--finish-radius", type=float, default=1.0)
    ap.add_argument("--map-starts",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "config", "map_starts.yaml"))
    args = ap.parse_args()

    files = (sorted(glob.glob(os.path.join(args.path, "*.csv")))
             if os.path.isdir(args.path) else [args.path])
    goals = parse_map_starts(args.map_starts)
    warn_arm_dist(goals, args.arm_dist, args.finish_radius)
    n = {"PASS": 0, "COLLISION": 0, "INCOMPLETE": 0, "EMPTY": 0}

    for f in files:
        base = os.path.basename(f)
        m = FNAME_RE.match(base)
        finish = finish_for_map(m.group("map"), goals, args.finish_radius) if m else None
        if not load_rows(f):
            print("  EMPTY       %s" % base); n["EMPTY"] += 1; continue
        s = summarize_run(f, args.collision_radius, finish, args.arm_dist)
        if s["collided"]:
            v, extra = "COLLISION", "min_dmin=%.2f" % s["min_dmin"]
        elif s["completed"]:
            v, extra = "PASS", "t=%.1fs  min_dmin=%.2f" % (s["t_course"], s["min_dmin"])
        else:
            v, extra = "INCOMPLETE", "min_dmin=%.2f (running or timed out)" % s["min_dmin"]
        if math.isfinite(s.get("min_ttc", math.inf)):
            extra += "  minTTC=%.2f" % s["min_ttc"]
        print("  %-11s %-42s %s" % (v, base, extra)); n[v] += 1

    print("\n  %d PASS  %d COLLISION  %d INCOMPLETE  %d EMPTY  (%d files)" %
          (n["PASS"], n["COLLISION"], n["INCOMPLETE"], n["EMPTY"], len(files)))


if __name__ == "__main__":
    main()
