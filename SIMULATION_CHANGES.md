# STLMPC Revision — Simulation Campaign: Code Changes & Run Guide

This document describes every change made to the repo to support the reviewer-
requested simulation studies (B1–B5), how to build and run the campaign, how to
turn the results into the manuscript's tables/figure, and the caveats to verify
in your ROS environment (nothing here was compiled/run on the authoring machine —
**do a single dry run before the full sweep**).

The manuscript edits live in the separate LaTeX folder (`root.tex`); see the
"Manuscript" section at the end for what to fill in.

---

## 1. What changed, and why

| File | Change | Reviewer item |
|---|---|---|
| `include/f1tenth_simulator/run_logger.h` | **New.** Header-only per-control-step CSV logger. | R2.2/2.3/2.4 (stats, TTC) |
| `node/navigation_STLMPC.cpp` | Telemetry hook (J_init/J_final, iters, solve time, timeout, d_min, adversary rel-pose for TTC), gated by `enable_logging`/`log_file`. | R2.2, R2.3, R2.4 |
| `node/navigation_STLMPC_vary_v.cpp` | Same telemetry hook, B2 ablation toggles (`enable_gvsteer`, `enable_gvobs`), V4 hard-clamp (`hard_clamp_v`), **and an MPPI solver mode** (`use_mppi`) that optimizes the identical objective+constraints by sampling instead of SQP. | R2.1, R2.2, R2.3, R2.4 |
| `node/navigation_FGM.cpp` | **New.** Follow-the-Gap baseline \cite{Sezer2012}: canonical FGM using the true geometric gap-centre angle (Eq. 8), goal-free (goal term dropped), no QP/MPC layers. Auto-built via the CMake `file(GLOB ...)`. | R2.1, R2.4 |
| `node/simulator.cpp` | Parameterized scripted adversary (straight/brake/swerve, optional 2nd), scan noise already existed (`scan_std_dev`) + new per-beam dropout, optional 2-vehicle scan-baking for occlusion. | R2.2, R2.4 (B5) |
| `params.yaml` | New campaign parameters (all default to the published behaviour, logging off). | — |
| `launch/campaign.launch` | **New.** Headless launch with all knobs as args. | — |
| `scripts/run_campaign.sh` | **New.** Batch driver: configs × maps × 10 seeds, randomized starts, per-run CSVs. | — |
| `scripts/aggregate_runs.py` | **New.** Per-run summary → per-config mean±std + success rate; emits LaTeX cells. | — |
| `scripts/compute_ttc.py` | **New.** Min-TTC from the logged adversary relative pose. | R2.3 |
| `scripts/plot_sensitivity.py` | **New.** B3 small-multiples → `Figures/fig_sensitivity.png`. | R2.3 |

**The b=0 clamp (Part 3.0) was already present** in both STLMPC nodes
(`if (std::abs(x[2]) < 1e-3) x[2] = std::copysign(1e-3, x[2]);`). Nothing to add —
just confirm it is committed and record the hash for the R1 letter.

**Defaults are non-invasive.** With `enable_logging:=0`, `adv_enable:=0`, and the
toggles at 1/1/0, every node reproduces the published behaviour, so hardware runs
and your existing sim demo are unaffected.

---

## 2. Build

Standard catkin build; the new node is picked up automatically by the existing
`file(GLOB node/*.cpp)` rule, and needs neither QuadProg nor NLopt.

```bash
cd ~/catkin_ws && catkin_make    # or catkin build
```

If the FGM node fails to find `run_logger.h`, confirm `include/` is on the include
path (it is, via `include_directories(include ...)` in `CMakeLists.txt`).

---

## 3. Run

```bash
source ~/catkin_ws/devel/setup.bash
OUT=~/stlmpc_logs SEEDS=10 RUN_SECONDS=60 \
  rosrun f1tenth_simulator run_campaign.sh      # or: bash scripts/run_campaign.sh
```

Before the full sweep, **do one dry run** and confirm two environment-specific
things the template cannot verify for you:

1. **Pose reset.** `run_campaign.sh` resets the ego to the seeded start via
   `rostopic pub /initialpose`. Confirm your `simulator.cpp` actually consumes
   `/initialpose` to move the ego (rviz "2D Pose Estimate" does this). If not,
   set the start pose another way (e.g., a param the simulator reads at reset).
   Also set a valid free-cell nominal start per map in the `START_X/Y/T` arrays.
2. **Run termination.** Runs are killed after `RUN_SECONDS`. For racing (map #3),
   either set `RUN_SECONDS` to comfortably exceed a lap, or pass a finish region
   to the aggregator (below) so `t_course` = time-to-finish, not wall duration.
3. **Nav mode.** The planner idles until the nav mux is selected (`nav_active`).
   The runner publishes `nav_key_char` ("n") to the keyboard topic to toggle it
   on, mirroring the `behavior_controller` path — confirm the vehicle actually
   starts moving after this in your dry run (check the `/key` and `/mux` topics if
   not, and that `keyboard_topic`/`nav_key_char` in `params.yaml` are unchanged).

### Config → knob mapping (authoritative)

- **B1** (sequential lines, horizon fixed at 16): `nMPC/kMPC` = `1/16`, `2/8`, `4/4`, map #1, `navigation_STLMPC`.
- **B2** (velocity constraints), map #3, `navigation_STLMPC_vary_v`:
  - V1 full `enable_gvsteer:=1 enable_gvobs:=1`
  - V2 no g_vsteer `enable_gvsteer:=0`
  - V3 no g_vobs `enable_gvobs:=0`
  - V4 hard clamp `hard_clamp_v:=1` (auto-disables both soft limits in-solver)
- **B3** (sensitivity): `d_dot_factor_STLMPC_vary_v`(λ_d̈²), `vel_factor_STLMPC_vary_v`(λ_v²), `safe_distance`(d_safe), `vel_beta`(β_v), `stop_distance_decay`(α_v), `theta_band_smooth`(s_θ).
- **B4 (baselines)**: `navigation_FGM` on maps #1–#3; **MPPI** vs **SQP** on map #3 via `navigation_STLMPC_vary_v` with `use_mppi:=1` / `:=0` (identical objective+constraints, so the comparison isolates the solver). MPPI knobs: `mppi_K`, `mppi_iters`, `mppi_lambda`, `mppi_sd_delta`, `mppi_sd_v`. MPPI is intentionally heavier than SQP — a larger `solve_time` is part of the expected result.
- **B5**: `navigation_STLMPC` with `adv_enable:=1` and `adv_maneuver:=brake|swerve`; `adv2_enable:=1` for the occluded two-vehicle case; `scan_std_dev` + `beam_dropout_prob` for R4.

---

## 4. Analyze → manuscript

```bash
# Ablation + robustness tables (LaTeX \ms{}{} cells to paste over the \TBD tokens)
python3 scripts/aggregate_runs.py ~/stlmpc_logs --latex --collision-radius 0.15 \
        --out summary.csv
# For racing t_course, add a finish region:
#   --finish-x <X> --finish-y <Y> --finish-radius 0.5

# B3 sensitivity figure referenced by root.tex
python3 scripts/plot_sensitivity.py summary.csv --metric mean_dmin \
        --out /path/to/LaTeX/Figures/fig_sensitivity.png

# Per-run TTC (dynamic runs)
python3 scripts/compute_ttc.py ~/stlmpc_logs/B5-R1_map2_seed00.csv
```

`aggregate_runs.py` prints, per config, a table row of
`success% & minₖd_min & d̄_min & Var(δ) & v̄` as `\ms{mean}{std}` cells. Paste
these over the `\TBD` placeholders in `root.tex`
(`\ref{tab:sim_ablation}`, `\ref{tab:sim_robust}`). The **solver-statistics**
sentence in the "Simulation Studies" subsection is filled from the
`mean_iters_mean`, `max_iters_mean`, `timeout_frac_mean`, and `j_ratio_mean`
columns of `summary.csv` (mean over all trials). The **min-TTC** column of the
robustness table comes from the `min_ttc_mean` column (dynamic rows only).

Per-run CSV columns: `t, x, y, theta, v_cmd, delta_cmd, d_min, fwd_min, J_init,
J_final, iters, solve_time, timeout, forcestop, success` (+ `det_active, det_x,
det_y, det_theta, det_v` on the dynamic node; `theta_head` on FGM).

---

## 5. Caveats / things to verify

- **Nothing here was compiled.** The C++ edits are surgical and scope-checked
  against the surrounding code, but please build once and skim the three modified
  nodes' new blocks (all are marked `// Revision`).
- **TTC / ego-adversary collision.** In sim the adversary is *not* in the
  occupancy map, so the simulator's own LiDAR collision check does not see it; the
  dynamic branch still plans around it via the TF→EKF detection. Ego-adversary
  proximity/TTC is therefore computed offline from the logged relative pose
  (`compute_ttc.py`), which is the intended metric. `d_min` on dynamic runs is the
  closest *map* obstacle; use `min_rel_dist`/`min_ttc` for the adversary.
- **B5-R3 (two vehicles).** The dynamic node tracks one adversary via TF. The
  second adversary affects the ego only through the optional scan-baking
  (`adv2_enable`, which bakes both outlines into the ego scan to create the
  occlusion). This demonstrates occlusion at the perception/planning level but
  does not run two independent EKF tracks; if a reviewer wants full multi-track,
  scope R3 to supplementary or extend the node's TF handler to a second child
  frame. The manuscript already lists R3 as a robustness row and points overflow
  to supplementary.
- **Swerve maneuver** integrates a yaw-rate impulse (`adv_swerve_rate` rad/s over
  `adv_swerve_dur` s); tune to taste for a visible ±0.3 rad step.
- **Randomized starts** use a deterministic seed→jitter so runs are reproducible.
- **Success/collision** default: collision if `min d_min < 0.15 m`; completed if
  not collided (and, if a finish region is given, reached it). Adjust
  `--collision-radius` to your vehicle footprint.

---

## 6. Manuscript (`root.tex`) — already edited, remaining to fill

Done (prose): abstract/conclusion claim-scoping, intro delta vs [PD]/[Miura]/[Morales],
degenerate-case sentence (II-A), initial-guess *feasible-by-construction* justification
(II-C, for R2.2), FGM + MPPI introduced in Setup, the full **Simulation Studies**
subsection (`\label{sec:simstudies}`) with Table `tab:sim_ablation`, Fig
`fig:sensitivity`, Table `tab:sim_robust` (now incl. MPPI/SQP rows), the solver-choice
+ statistics paragraph (MPPI-vs-SQP on the identical problem), and the
Discussion/Limitations subsection. `[Date]` and bios are already commented out.

**Section VI was reordered to simulation-first**: `\section{Performance Evaluation}`
now runs Setup → Simulation Studies → the three hardware experiments
(`sec:exp1`/`sec:exp2`/`sec:exp3`) → Discussion and Limitations. Metric definitions
live once in the sim subsection; the hardware text refers back to them. The
single-run-hardware vs 10-seed-sim distinction is stated explicitly so the two are
never conflated.

Remaining: replace every `\TBD` (grep for it) with numbers from `aggregate_runs.py`,
drop `fig_sensitivity.png` into `Figures/`, then recompile and **check page count
≤ 14** (move the lowest-value float to supplementary if over).
