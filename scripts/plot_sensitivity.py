"""plot_sensitivity.py -- B3 one-at-a-time sensitivity small-multiples figure.

Consumes the summary.csv produced by aggregate_runs.py and renders the
Figures/fig_sensitivity.png referenced by root.tex (Fig. \ref{fig:sensitivity}).

Config-naming convention for B3 runs (encoded in the log-file names, recovered by
aggregate_runs.py as the CONFIG token):

    B3-<param>-<value>_map<M>_seed<NN>.csv

e.g.  B3-lambda_dd-30_map1_seed04.csv,  B3-beta_v-3.3_map3_seed09.csv

Each panel sweeps one <param>; the shaded band is metric mean +/- std over the 10
seeds at each value, and the dashed line marks the nominal value from Table I.

Requires matplotlib + numpy.
"""
import argparse, csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# param token -> (LaTeX-ish label, nominal value from Table I)
PARAMS = {
    "lambda_dd": (r"$\lambda_{\dot d^2}$", 30.0),
    "lambda_v":  (r"$\lambda_{v^2}$", 1.0),
    "d_safe":    (r"$d_\mathrm{safe}$ (m)", 1.25),
    "beta_v":    (r"$\beta_v$ (m$^{-1}$)", 10.0),
    "alpha_v":   (r"$\alpha_v$ (m)", 0.5),
    "s_theta":   (r"$s_\theta$ (rad$^{-1}$)", 200.0),
}


def load_summary(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def parse(rows, metric):
    """Return {param: [(value, mean, std), ...]} for configs named B3-<param>-<value>."""
    data = {}
    for r in rows:
        cfg = r["config"]
        if not cfg.startswith("B3-"):
            continue
        body = cfg[3:]
        # split on last '-' so param tokens may contain underscores but not '-'
        if "-" not in body:
            continue
        param, val = body.rsplit("-", 1)
        try:
            val = float(val)
            mu = float(r[metric + "_mean"])
            sd = float(r[metric + "_std"])
        except (KeyError, ValueError):
            continue
        data.setdefault(param, []).append((val, mu, sd))
    for p in data:
        data[p].sort(key=lambda t: t[0])
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("summary_csv")
    ap.add_argument("--metric", default="mean_dmin",
                    help="metric column stem (default mean_dmin)")
    ap.add_argument("--ylabel", default=r"$\bar d_\mathrm{min}$ (m)")
    ap.add_argument("--out", default="Figures/fig_sensitivity.png")
    args = ap.parse_args()

    rows = load_summary(args.summary_csv)
    data = parse(rows, args.metric)

    order = [p for p in PARAMS if p in data] or list(data.keys())
    n = max(1, len(order))
    ncol = 3
    nrow = int(math.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(7.0, 2.1 * nrow), squeeze=False)

    for k, param in enumerate(order):
        ax = axes[k // ncol][k % ncol]
        pts = data[param]
        xs = np.array([p[0] for p in pts])
        mu = np.array([p[1] for p in pts])
        sd = np.array([p[2] for p in pts])
        ax.plot(xs, mu, "-o", color="#1f77b4", ms=3, lw=1.2)
        ax.fill_between(xs, mu - sd, mu + sd, color="#1f77b4", alpha=0.20, lw=0)
        label, nominal = PARAMS.get(param, (param, None))
        if nominal is not None:
            ax.axvline(nominal, ls="--", color="0.5", lw=0.9)
        ax.set_xlabel(label, fontsize=9)
        ax.set_ylabel(args.ylabel, fontsize=8)
        ax.tick_params(labelsize=7)
        ax.margins(x=0.05)

    # hide unused axes
    for k in range(len(order), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")

    fig.tight_layout()
    fig.savefig(args.out, dpi=200, bbox_inches="tight")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
