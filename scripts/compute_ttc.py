"""compute_ttc.py -- minimum Time-to-Collision for a dynamic STLMPC run.

The dynamic STLMPC node (navigation_STLMPC) logs, per control step, the tracked
adversary's pose/speed in the ego base frame (det_x, det_y, det_theta, det_v)
alongside the ego command (v_cmd). With the ego at the origin heading +x in its
own base frame, TTC at each step is the closing-time of the relative position
under the relative velocity:

    p      = (det_x, det_y)                          # adversary position, ego frame
    v_adv  = det_v * (cos det_theta, sin det_theta)  # adversary velocity, ego frame
    v_ego  = (v_cmd, 0)                              # ego velocity, ego frame
    v_rel  = v_adv - v_ego
    closing = -(p . v_rel) / |p|                     # >0 when distance shrinking
    TTC    = |p| / closing        (only when closing > 0, else +inf)

min TTC over the run is the reported safety-margin metric. Rows without an active
detection (det_active == 0) are skipped. Pure Python stdlib; no dependencies.
"""
import math


def step_ttc(row):
    """TTC for one logged row (dict of floats), or +inf if not closing / no det."""
    if float(row.get("det_active", 0)) < 0.5:
        return math.inf
    px, py = float(row["det_x"]), float(row["det_y"])
    dist = math.hypot(px, py)
    if dist < 1e-6:
        return 0.0
    dth, dv = float(row["det_theta"]), float(row["det_v"])
    vego = float(row.get("v_cmd", 0.0))
    vrx = dv * math.cos(dth) - vego
    vry = dv * math.sin(dth)
    closing = -(px * vrx + py * vry) / dist
    if closing <= 1e-6:
        return math.inf
    return dist / closing


def min_ttc(rows):
    """Minimum TTC (s) over a list of row dicts; math.inf if never closing."""
    vals = [step_ttc(r) for r in rows]
    vals = [v for v in vals if math.isfinite(v)]
    return min(vals) if vals else math.inf


def min_rel_distance(rows):
    """Closest ego-adversary distance (m) over the run; math.inf if no detections."""
    ds = []
    for r in rows:
        if float(r.get("det_active", 0)) >= 0.5:
            ds.append(math.hypot(float(r["det_x"]), float(r["det_y"])))
    return min(ds) if ds else math.inf


if __name__ == "__main__":
    import argparse, csv
    ap = argparse.ArgumentParser(description="Min TTC for one run CSV.")
    ap.add_argument("csv_file")
    args = ap.parse_args()
    with open(args.csv_file, newline="") as f:
        rows = list(csv.DictReader(f))
    print("min_TTC (s):      %.3f" % min_ttc(rows))
    print("min_rel_dist (m): %.3f" % min_rel_distance(rows))
