# STLMPC Simulation Campaign — Onboarding & Run Guide

This README is the single source of truth for running the STLMPC revision
**simulation campaign** (paper tables/figures B1–B5) and finishing the paper.
It is written so a fresh agent or a new device can continue with no prior context.

The **paper draft** (`root.tex`) lives *outside* this code repo. The campaign here
produces the numbers that fill its `\TBD` cells (see "What feeds the paper" below).

---

## 0. TL;DR workflow

```bash
# 1. build
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2. verify per-map ego/goal look right in rviz (once per map)
roslaunch f1tenth_simulator campaign.launch gui:=true map_name:=map4   # look, don't need to drive

# 3. smoke test: 1 seed of everything, confirm all PASS
SEEDS=1 bash scripts/run_campaign.sh
python3 scripts/check_run.py

# 4. full sweep, one study group at a time (resumable)
RESUME=1 SEEDS=10 ONLY="B1" bash scripts/run_campaign.sh
RESUME=1 SEEDS=10 ONLY="B2" bash scripts/run_campaign.sh
RESUME=1 SEEDS=10 ONLY="B3" bash scripts/run_campaign.sh
RESUME=1 SEEDS=10 ONLY="B4" bash scripts/run_campaign.sh
RESUME=1 SEEDS=10 ONLY="B5" bash scripts/run_campaign.sh

# 5. aggregate into paper-ready numbers (arm-dist MUST match campaign.launch goal_arm_dist)
python3 scripts/aggregate_runs.py ~/stlmpc_logs --latex --collision-radius 0.15 --arm-dist 5.0
python3 scripts/plot_sensitivity.py            # Fig. sensitivity from the B3-* runs

# 6. hand the agent: summary.csv (+ the --latex block) and Figures/fig_sensitivity.png
```

---

## 1. What this campaign is

STLMPC is evaluated in the `f1tenth_simulator` on 5 maps. The **scenario type is set
by the planner + adversary config, not the map**:

- **Constant-v STLMPC** (`navigation_STLMPC`) — static/dynamic.
- **Variable-v STLMPC** (`navigation_STLMPC_vary_v`) — racing.
- **FGM** (`navigation_FGM`) — gap-following baseline.
- **MPPI** — `navigation_STLMPC_vary_v use_mppi:=1`, the same objective/constraints solved by sampling instead of SQP.

Each config × map is run over **10 randomized-start seeds**; metrics are reported as
mean ± std with a success rate.

Map roles: **map4** switchback (B1/B2/B3, MPPI/SQP) · **map1** 2nd cornered map + noise + FGM ·
**map2** open/dynamic adversary (B5 R1/R2/R3) · **map3/map5** unseen generalization.

---

## 2. What was built (revision infrastructure)

All of the following is committed in this repo:

| Area | Files | What it does |
|---|---|---|
| Per-map ego + goal (single source of truth) | `config/map_starts.yaml` | ego spawn pose and course goal (x,y,radius) per map |
| Campaign launch | `launch/campaign.launch` | one node graph; all knobs are args (see §5) |
| Simulator | `node/simulator.cpp` | ego spawn; **adversary anchored to nav-enable**; **ego-relative, per-maneuver adversary placement**; **goal-based completion + timeout** (`/run_complete`); rviz **goal markers**; phantom vehicle gated off when `adv_enable=0`; `move_base_wait` |
| Hands-free start | `node/behavior_controller.cpp` | `auto_nav` auto-enables nav after a delay |
| Telemetry | `include/f1tenth_simulator/run_logger.h` | per-control-step CSV |
| Batch driver | `scripts/run_campaign.sh` | config×map×seed sweep; `RESUME`; setsid + SIGINT trap teardown |
| Aggregation | `scripts/aggregate_runs.py` | per-map goals + arming + metric truncation → `summary.csv` + LaTeX cells |
| TTC | `scripts/compute_ttc.py` | min Time-to-Collision (dynamic runs) |
| Per-run status | `scripts/check_run.py` | quick PASS / COLLISION / INCOMPLETE / EMPTY |
| Sensitivity fig | `scripts/plot_sensitivity.py` | Fig. sensitivity from `B3-*` runs |
| rviz | `launch/simulator.rviz` | includes the `/goal_marker` display |
| Defaults | `params.yaml` | logging, campaign, MPPI, `auto_nav`, goal defaults |

### Key design decisions (why things are the way they are)
- **Adversary timeline is anchored to nav-enable**, not sim launch, and the ego +
  adversary are snapped to their start poses the instant nav turns on — so the ego
  always witnesses the full maneuver and runs are reproducible.
- **Adversary is placed relative to the ego** (forward/left/heading offset), so it
  stays correctly positioned for any map/ego-heading. Geometry is chosen per maneuver:
  `brake` = front-right lead into the bend (`3.5 m fwd, −2.0 m left, −0.6 rad`),
  `swerve` = ~90° port crosser, `straight`/occluded = crosser(s).
- **Completion**: a run ends (success) when the ego reaches the per-map goal *after*
  first leaving it by `goal_arm_dist`; else it times out at `run_timeout` (failure).
  Online completion in the sim = offline completion in `aggregate_runs.py` **only if
  `--arm-dist` matches `goal_arm_dist`** (currently 5.0 — see §6).
- **use_neural_net auto-on for dynamic runs** (`default="$(arg adv_enable)"`) so the
  planner actually plans around the tracked adversary.

---

## 3. Current status

**Done:** all code/infrastructure above; the pipeline runs end-to-end (a `B1-A1-map4`
seed completes and logs correctly).

**Must verify before the full 10-seed sweep (per-map goal correctness):**
- `map_starts.yaml` goals for **map3, map4, map5** are still marked `TODO(capture)`.
  map4 drives most of the paper (B1/B2/B3/B4), so its goal being wrong poisons two
  tables. In the `SEEDS=1` smoke test, every finished run must read **PASS** in
  `check_run.py` (not `INCOMPLETE`/timeout). If a map times out, capture its real
  ego/goal (see §7) before scaling to 10 seeds.
- map3's goal is only ~2.8 m from its ego start while `goal_arm_dist=5` — if map3
  shows `INCOMPLETE`, lower its arm distance or fix the goal.
- map2 dynamic runs (R1/R2/R3) must PASS **and** show a finite `minTTC` (adversary
  engaged).

**Not started:** filling the paper `\TBD` cells (needs `summary.csv`).

---

## 4. What feeds the paper (`root.tex`)

Run `aggregate_runs.py` once over the whole log dir; it emits every config. Map the
rows to the paper as follows.

**`Table sim_ablation`** (Map #4, B1 also replicated on Map #1):
- Upper block B1: `B1-A1-*` (nMPC=1,kMPC=16), `B1-A2-*` (2,8), `B1-A3-*` (4,4) — `navigation_STLMPC`.
- Lower block B2: `B2-V1..V4` — `navigation_STLMPC_vary_v`.
- Columns: Success%, `min_dmin`, `mean_dmin`, `var_delta`, `mean_v`.

**`Fig. sensitivity`** (Map #4): `B3-*` (6 params × 5 values) → `plot_sensitivity.py` → `Figures/fig_sensitivity.png`.

**`Table sim_robust`**:
- `B5-R1` brake, `B5-R2` swerve, `B5-R3` two-veh occluded — Map #2 (dynamic).
- `B5-R4-{0.01,0.03,0.05}` scan noise+dropout — Map #1.
- `B4-FGM`, `B4-MPPI`, `B4-SQP` — Map #4.
- Generalization `B5-GEN-m3`, `B5-GEN-m5`.
- Columns: Success%, `min_dmin`, `mean_dmin`, `minTTC` (dynamic rows only; `---` elsewhere), `mean_v`.

**Solver-stats paragraph**: `mean_iters`, `max_iters`, `timeout_frac`, `j_ratio` — take
these from the **SQP/STLMPC** configs. **Do NOT** use MPPI's `iters`/`timeout` here
(fixed `mppi_K*mppi_iters`, and "timeout" just means >50 ms — not comparable to SQP).
MPPI only feeds the `sim_robust` clearance/objective comparison.

---

## 5. campaign.launch knobs (all overridable on the command line)

- `map_name` (map1..map5) → derives map path and selects the `map_starts.yaml` block.
- `planner` = `navigation_STLMPC` | `navigation_STLMPC_vary_v` | `navigation_FGM`.
- `nMPC`,`kMPC` (B1); `enable_gvsteer`,`enable_gvobs`,`hard_clamp_v` (B2);
  `d_dot_factor_STLMPC_vary_v`,`vel_factor_STLMPC_vary_v`,`safe_distance`,`vel_beta`,
  `stop_distance_decay`,`theta_band_smooth` (B3).
- `use_mppi`,`mppi_K`,`mppi_iters`,`mppi_lambda`,`mppi_sd_delta`,`mppi_sd_v` (B4/MPPI).
- `adv_enable`,`adv_maneuver` (`straight|brake|swerve`),`adv2_enable`,`scan_std_dev`,`beam_dropout_prob` (B5).
- `adv_relative` (1=ego-relative), `adv_forward/adv_left/adv_rel_theta` (+`adv2_*`) to retune placement (`-999`=per-maneuver default).
- `use_neural_net` (default = `adv_enable`).
- `auto_nav`,`auto_nav_delay`; `goal_arm_dist`,`run_timeout`,`shutdown_on_goal`;
  `ego_jitter_x/y/theta`; `gui` (rviz).
- `move_base_wait` (campaign sets 0.1 — no move_base in the campaign graph).

---

## 6. Running the sweep & aggregating

- `run_campaign.sh` env vars: `OUT` (default `~/stlmpc_logs`), `SEEDS` (10), `RUN_SECONDS`
  (60 = per-run timeout), `ONLY` (e.g. `"B1 B2"`), `RESUME` (1 = skip finished seeds),
  `MIN_ROWS` (50 = "finished" threshold for RESUME).
- Log names: `<CONFIG>_map<M>_seed<NN>.csv`.
- **Runs are strictly sequential — do NOT parallelize on a memory-limited VM.**
- Aggregate with **`--arm-dist 5.0`** to match `goal_arm_dist:=5.0` in `campaign.launch`
  (if you change one, change both), and `--collision-radius 0.15` (paper definition).

---

## 7. Capturing a map's ego / goal in rviz (~30 s)

```bash
roslaunch f1tenth_simulator campaign.launch gui:=true map_name:=map4
```
- The green **GOAL** sphere + ring (radius = `goal_radius`) shows the current goal.
- Use rviz **2D Pose Estimate** to drop the ego (or the finish point); the simulator
  prints a paste-ready `ego_x/ego_y/ego_theta` block to its console → paste into that
  map's block in `config/map_starts.yaml` (`ego_*` for start, `goal_*` for finish).
- For a **loop**, set `goal_* = ego_*`. For **point-to-point**, set `goal_*` to the finish end.

---

## 8. Visualizing a single run (incl. one specific seed)

Just run `campaign.launch` directly with `gui:=true` (that runs ONE config, not the sweep).
To reproduce a specific batch seed's exact randomized start, compute its jitter the same
way `run_campaign.sh` does and pass it as `ego_jitter_*`. Example — **B1-A1 (map4), seed 9**:

```bash
sd=9
dx=$(awk -v s=$sd 'BEGIN{srand(s*97+1*7919); print 0.3*(2*rand()-1)}')
dy=$(awk -v s=$sd 'BEGIN{srand(s*97+2*7919); print 0.3*(2*rand()-1)}')
dt=$(awk -v s=$sd 'BEGIN{srand(s*97+3*7919); print 0.1745*(2*rand()-1)}')
roslaunch f1tenth_simulator campaign.launch gui:=true \
  map_name:=map4 planner:=navigation_STLMPC nMPC:=1 kMPC:=16 \
  ego_jitter_x:=$dx ego_jitter_y:=$dy ego_jitter_theta:=$dt \
  auto_nav:=1
```
Swap the `planner`/`nMPC`/`kMPC`/`adv_*` args to match whichever config you want to watch
(e.g. `B1-A3` = `nMPC:=4 kMPC:=4`; `B5-R1` = `planner:=navigation_STLMPC adv_enable:=1 adv_maneuver:=brake map_name:=map2`).
Watch the console for `COURSE COMPLETE ... in T s`. If you don't need the exact seed, omit
the `ego_jitter_*` lines (nominal start).

---

## 9. Troubleshooting (VM)

- **Rebuild after pulling** — most fixes are C++ (`catkin_make`). Stale binaries silently ignore them.
- **CPU ~100% during a run** = the solver; normal (esp. MPPI). Give the VM 3–4 vCPUs for headroom.
- **Runs crash** = almost always OOM. `dmesg | grep -i oom` to confirm. Fix: VM ≥6 GB RAM
  **+ a swapfile** (`sudo fallocate -l 4G /swapfile; sudo chmod 600 /swapfile; sudo mkswap /swapfile; sudo swapon /swapfile`).
- **CPU stays pinned after Ctrl-C / between runs** = orphaned nodes. The batch now traps
  SIGINT and kills the run's process group; if you still see leftovers:
  `pkill -9 -f campaign.launch; pkill -9 -f f1tenth_simulator; pkill -9 -f navigation_; pkill -9 rosmaster`.
- **A sweep died partway** → re-run with `RESUME=1` (keeps finished seeds).
- **`check_run.py`/`aggregate` crash on a file** — already tolerated (NUL-safe); a partial
  file just reads as INCOMPLETE/EMPTY.
- **`minTTC` on a non-B5 run** — shouldn't happen after the phantom gating; if it does, rebuild.

---

## 10. Definition of done (paper)

1. All maps PASS in the `SEEDS=1` smoke test (goals verified).
2. `SEEDS=10` sweep complete for B1–B5 (`check_run.py` shows expected PASS/COLLISION spread).
3. `summary.csv` produced with `--arm-dist 5.0 --collision-radius 0.15`.
4. `Figures/fig_sensitivity.png` produced from B3.
5. Hand `summary.csv` (+ LaTeX cells) to the agent → it fills `Table sim_ablation`,
   `Table sim_robust`, the sensitivity text, and the solver-stats paragraph in `root.tex`.
```
