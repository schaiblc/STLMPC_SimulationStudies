#!/usr/bin/env bash
# run_campaign.sh -- batch driver for the STLMPC revision simulation campaign.
#
# For each configuration x map x seed it: (1) computes a deterministic randomized
# start offset (+/-0.3 m, +/-10 deg) from the seed, (2) launches campaign.launch
# headless with the config's param overrides, the map's start poses/goal (from
# config/map_starts.yaml), auto_nav and shutdown_on_goal, and a per-run log file,
# (3) waits for the run to self-terminate on goal-completion or run_timeout. Log
# files are named <CONFIG>_map<M>_seed<NN>.csv so scripts/aggregate_runs.py can
# recover config/map/seed.
#
# This is a TEMPLATE: do a single dry run first (see the one-at-a-time guide) and
# make sure each map's ego/goal in map_starts.yaml are captured. The config->param
# mapping below is the authoritative, non-obvious part.
#
# Requires a sourced ROS Noetic workspace (roslaunch, rostopic on PATH).
set -u

PKG=f1tenth_simulator
OUT=${OUT:-$HOME/stlmpc_logs}          # where CSVs are written
SEEDS=${SEEDS:-10}                      # seeds per configuration
RUN_SECONDS=${RUN_SECONDS:-60}          # per-run timeout: failure if goal not reached by then
# RESUME=1 skips seeds already finished (>= MIN_ROWS rows) so a crashed/OOM'd sweep
# can be re-run without redoing completed work. Runs are strictly sequential (one at
# a time) -- do not parallelize on a memory-constrained VM.
RESUME=${RESUME:-0}
MIN_ROWS=${MIN_ROWS:-50}
mkdir -p "$OUT"

# Because each run is launched with setsid (its own process group), a Ctrl-C on this
# script would NOT reach the running roslaunch -- it would keep spinning as an orphan
# at ~100% CPU. This trap force-kills the in-flight run's whole group on interrupt.
CUR=""   # process-group id (== leader pid) of the in-flight run
cleanup() {
  trap - INT TERM
  if [ -n "$CUR" ]; then
    echo "" >&2; echo "Interrupted -- stopping current run ($CUR)..." >&2
    kill -INT  -"$CUR" 2>/dev/null
    timeout 5 bash -c "while kill -0 -\"$CUR\" 2>/dev/null; do sleep 0.5; done"
    kill -KILL -"$CUR" 2>/dev/null
  fi
  exit 130
}
trap cleanup INT TERM

# SIMULATION maps only (map1..map5), selected by basename via the map_name arg.
# The per-map ego/adversary/goal start poses now live in config/map_starts.yaml
# (single source of truth); this script only adds the per-seed randomized offset.
# Map roles: map4 switchback (B1/B2/B3, MPPI/SQP), map1 second cornered map + R4,
# map2 open/dynamic (B5 R1/R2/R3), map3/map5 unseen generalization.

# deterministic pseudo-random jitter in [-1,1] from (seed, salt)
jit() { awk -v s="$1" -v k="$2" 'BEGIN{srand(s*97+k*7919); print 2*rand()-1}'; }

# Optional subset filter: set ONLY="B1 B2 B4" to run only those study prefixes.
# e.g. ONLY="B1 B2 B4" bash run_campaign.sh   (core ablations + baselines first)
ONLY=${ONLY:-}

# run_one CONFIG MAPKEY PLANNER "extra roslaunch args..."
run_one() {
  local cfg="$1" mapk="$2" planner="$3"; shift 3
  local extra="$*"
  if [ -n "$ONLY" ]; then
    local keep=0; for p in $ONLY; do case "$cfg" in ${p}*) keep=1;; esac; done
    [ "$keep" = 0 ] && return 0
  fi
  for ((sd=0; sd<SEEDS; sd++)); do
    local seed=$(printf "%02d" "$sd")
    local log="$OUT/${cfg}_${mapk}_seed${seed}.csv"

    # Resume: skip a seed already run to completion (>= MIN_ROWS logged rows), so a
    # crash/OOM part-way through the sweep is recovered by simply re-running the same
    # command -- finished runs are kept, only empty/partial ones are redone.
    #   RESUME=1 SEEDS=10 ONLY="B3" bash scripts/run_campaign.sh
    if [ "${RESUME:-0}" = 1 ] && [ -f "$log" ]; then
      local have; have=$(wc -l < "$log" 2>/dev/null || echo 0)
      if [ "${have:-0}" -ge "${MIN_ROWS:-50}" ]; then
        echo "--- skip (done, $have rows): ${cfg}_${mapk}_seed${seed}"
        continue
      fi
    fi

    local dx dy dt
    dx=$(awk -v j="$(jit "$sd" 1)" 'BEGIN{print 0.3*j}')
    dy=$(awk -v j="$(jit "$sd" 2)" 'BEGIN{print 0.3*j}')
    dt=$(awk -v j="$(jit "$sd" 3)" 'BEGIN{print 0.1745*j}')  # 10 deg in rad

    echo ">>> $cfg $mapk seed=$seed  jitter=($dx,$dy,$dt)"
    # The ego/adversary/goal come from map_starts.yaml (selected by map_name); the
    # seed offset is passed as ego_jitter_*. auto_nav starts navigation hands-free,
    # and shutdown_on_goal ends the launch on completion or after run_timeout, so no
    # /initialpose or /key publishing is needed and completed runs stop immediately.
    # setsid => the launch is its own process group, so we can hard-kill every node
    # at once if one hangs. --sigint/--sigterm-timeout cap how long roslaunch waits
    # for a busy node (e.g. the solver mid-optimize) before escalating to SIGKILL,
    # which avoids the multi-second CPU spike on a slow shutdown.
    setsid roslaunch $PKG campaign.launch \
        --sigint-timeout=3 --sigterm-timeout=3 \
        map_name:="$mapk" planner:="$planner" \
        log_file:="$log" enable_logging:=1 \
        auto_nav:=1 auto_nav_delay:=3.0 \
        shutdown_on_goal:=1 run_timeout:="$RUN_SECONDS" \
        ego_jitter_x:="$dx" ego_jitter_y:="$dy" ego_jitter_theta:="$dt" \
        $extra \
        >/dev/null 2>&1 &
    local lpid=$!
    CUR=$lpid   # expose to the SIGINT trap so Ctrl-C tears this run down

    # Normally the simulator ends the launch itself on goal/run_timeout. If that
    # doesn't happen within a grace window, force-kill the whole process group.
    local waited=0 cap
    cap=$(awk -v r="$RUN_SECONDS" 'BEGIN{printf "%d", r+25}')
    while kill -0 $lpid 2>/dev/null; do
      sleep 1; waited=$((waited+1))
      if [ "$waited" -ge "$cap" ]; then
        echo "!!! run exceeded cap, killing $lpid" >&2
        kill -INT  -"$lpid" 2>/dev/null
        sleep 4
        kill -KILL -"$lpid" 2>/dev/null
        break
      fi
    done

    # --- Staggered, bounded teardown (replaces the plain `wait $lpid`) ---
    # Kill the heaviest node first and give it a short window, THEN take down
    # the rest of the group. Every step has a hard timeout so this function
    # can NEVER block indefinitely, regardless of what the nodes do.
    # Kill the run's ACTUAL planner (the node is named after $planner), not a
    # hardcoded /navigation_STLMPC -- that was a no-op for the vary_v/FGM/MPPI runs,
    # leaving the heaviest node alive for the whole staggered teardown below.
    rosnode kill "/$planner" >/dev/null 2>&1
    timeout 5 bash -c "while kill -0 -\"$lpid\" 2>/dev/null; do sleep 0.5; done"

    # Whatever's left in the group (if anything) gets SIGINT, bounded to 5s.
    kill -0 -"$lpid" 2>/dev/null && kill -INT -"$lpid" 2>/dev/null
    timeout 5 bash -c "while kill -0 -\"$lpid\" 2>/dev/null; do sleep 0.5; done"

    # Anything STILL alive after both windows gets force-killed, no exceptions.
    if kill -0 -"$lpid" 2>/dev/null; then
      echo "!!! group $lpid would not die, SIGKILL" >&2
      kill -KILL -"$lpid" 2>/dev/null
      sleep 1
    fi

    # Don't call plain `wait` here -- it can block on a detached setsid group.
    wait $lpid 2>/dev/null
    CUR=""
    sleep 1
  done
}

# ----------------------------------------------------------------------------
# Map roles (scenario type comes from the planner/adversary config, NOT the map):
#   map4 = switchback (tight corners) -> corner-dependent ablations B1/B2/B3, MPPI/SQP
#   map1 = has corners  -> second curvy map for the sequential-line ablation, noise, FGM
#   map2 = open/corridor -> dynamic adversary encounters (B5 R1/R2/R3)
#   map3, map5 = unseen (not in ablations) -> generalization
# ----------------------------------------------------------------------------

# B1 -- sequential-vs-single tracking line (fixed 1.6 s horizon), constant-v STLMPC.
# Run on the switchback (map4) where corner clipping is clearest, and replicated on a
# second curvy map (map1) for a stronger, multi-map ablation (config token carries the map).
for mk in map4 map1; do
  run_one "B1-A1-$mk" $mk navigation_STLMPC "nMPC:=1 kMPC:=16"
  run_one "B1-A2-$mk" $mk navigation_STLMPC "nMPC:=2 kMPC:=8"
  run_one "B1-A3-$mk" $mk navigation_STLMPC "nMPC:=4 kMPC:=4"
done

# B1b -- the same sequential-line ablation at a LONGER HORIZON DISTANCE. B1 runs a
# 1.6 s horizon at v=1.5 m/s, i.e. only 2.4 m of travel, over which a single tracking
# line covers the corridor about as well as two (A1 and A2 come out statistically
# indistinguishable). B1b grows the horizon distance two ways, so the switchback bend
# falls inside the horizon and the sequential structure has something to exploit:
#   -H  lengthen the horizon: nMPC*kMPC = 32 (3.2 s = 4.8 m), problem size doubles
#   -V  raise the speed:      v = 2.5 m/s (1.6 s = 4.0 m), problem size UNCHANGED,
#       so -V isolates horizon distance with no solver-cost confound.
# Smoke-test both at SEEDS=1 first: -H may push solve time into the 50 ms cap, and -V
# may be infeasible for the constant-v planner on the switchback (it cannot slow down).
for mk in map4; do
  run_one "B1b-H-A1-$mk" $mk navigation_STLMPC "nMPC:=1 kMPC:=32"
  run_one "B1b-H-A2-$mk" $mk navigation_STLMPC "nMPC:=2 kMPC:=16"
  run_one "B1b-H-A3-$mk" $mk navigation_STLMPC "nMPC:=4 kMPC:=8"
  run_one "B1b-V-A1-$mk" $mk navigation_STLMPC "nMPC:=1 kMPC:=16 vehicle_velocity:=2.5"
  run_one "B1b-V-A2-$mk" $mk navigation_STLMPC "nMPC:=2 kMPC:=8  vehicle_velocity:=2.5"
  run_one "B1b-V-A3-$mk" $mk navigation_STLMPC "nMPC:=4 kMPC:=4  vehicle_velocity:=2.5"
done

# B7 -- DISABLED. Intent was to repeat the B1 sequential-line ablation on the physical
# course geometry, to test whether two lines beat one at the nominal 16-sample horizon
# on the tighter, more turn-dense courses the hardware ran on. The experiment*.pgm
# grids cannot serve this purpose:
#   * they are ~98% UNKNOWN space (1.7% free, 0.2% occupied) -- sparse AMCL grids, not
#     drivable maps. Driven as-is the simulator treats unknown as obstacle, the planner
#     sees hazard everywhere and the vehicle crawls at 0.23 m/s (verified).
#   * filling unknown->free instead leaves the sparse wall segments floating in open
#     space (free component 108-175 m2, 2.0 m clearance at the most open cell), so the
#     vehicle drives between wall fragments rather than following the course.
# Reconstructing the real layout needs the course geometry, not an automatic fill. If a
# proper occupancy map of a physical course becomes available, add it to map_starts.yaml
# and re-enable the block below (stop_distance:=0.5 is the hardware value).
#
# for mk in experiment1 experiment3; do
#   run_one "B7-A1-$mk" $mk navigation_STLMPC "nMPC:=1 kMPC:=16 stop_distance:=0.5"
#   run_one "B7-A2-$mk" $mk navigation_STLMPC "nMPC:=2 kMPC:=8  stop_distance:=0.5"
#   run_one "B7-A3-$mk" $mk navigation_STLMPC "nMPC:=4 kMPC:=4  stop_distance:=0.5"
# done

# B2 -- velocity-constraint ablation, switchback map (map4), variable-velocity node
run_one B2-V1 map4 navigation_STLMPC_vary_v "enable_gvsteer:=1 enable_gvobs:=1 hard_clamp_v:=0"
run_one B2-V2 map4 navigation_STLMPC_vary_v "enable_gvsteer:=0 enable_gvobs:=1 hard_clamp_v:=0"
run_one B2-V3 map4 navigation_STLMPC_vary_v "enable_gvsteer:=1 enable_gvobs:=0 hard_clamp_v:=0"
run_one B2-V4 map4 navigation_STLMPC_vary_v "enable_gvsteer:=0 enable_gvobs:=0 hard_clamp_v:=1"

# B3 -- one-at-a-time sensitivity (5 values/param) on the switchback map (map4).
# Config token = B3-<param>-<value>.
for v in 10 17 30 52 90;      do run_one "B3-lambda_dd-$v" map4 navigation_STLMPC_vary_v "d_dot_factor_STLMPC_vary_v:=$v"; done
for v in 0.33 0.58 1 1.7 3;   do run_one "B3-lambda_v-$v"  map4 navigation_STLMPC_vary_v "vel_factor_STLMPC_vary_v:=$v"; done
for v in 0.75 1 1.25 1.6 2;   do run_one "B3-d_safe-$v"    map4 navigation_STLMPC_vary_v "safe_distance:=$v"; done
for v in 3.3 5.8 10 17 30;    do run_one "B3-beta_v-$v"    map4 navigation_STLMPC_vary_v "vel_beta:=$v"; done
for v in 0.17 0.29 0.5 0.87 1.5; do run_one "B3-alpha_v-$v" map4 navigation_STLMPC_vary_v "stop_distance_decay:=$v"; done
for v in 67 115 200 350 600;  do run_one "B3-s_theta-$v"   map4 navigation_STLMPC_vary_v "theta_band_smooth:=$v"; done

# B4 -- baselines. FGM (gap heuristic) spread across layouts (static, open, switchback);
# MPPI (sampling optimizer on the SAME STLMPC objective+constraints) vs SQP on map4.
# The config token must carry the map for any study that spans several maps (as B1
# does): aggregate_runs.py groups by config alone, so a shared "B4-FGM" token would
# average the three layouts into one meaningless row.
run_one B4-FGM-map1 map1 navigation_FGM ""
run_one B4-FGM-map2 map2 navigation_FGM ""
run_one B4-FGM-map4 map4 navigation_FGM ""
# MPPI is given its best-faith configuration: warm-started nominal sequence and a
# temperature scaled to the sample cost spread. Both were isolated one-at-a-time;
# warm-starting raises mean speed 0.53->0.72 m/s and the adaptive temperature a
# further 0.72->1.22, so reporting the untuned default would understate the baseline.
run_one B4-MPPI map4 navigation_STLMPC_vary_v "use_mppi:=1 mppi_warm:=1 mppi_lambda:=-1"
run_one B4-SQP  map4 navigation_STLMPC_vary_v "use_mppi:=0"   # matched SQP reference

# B8 -- MPPI solution-quality PROBE (R2.2: is the SQP solution a poor local optimum?).
# SQP drives; at every control step the identical problem instance is ALSO solved by
# MPPI from the same Algorithm-2 initial guess, and both objectives are logged
# (J_final, J_mppi). Unlike the closed-loop B4-MPPI race, this cannot be confounded by
# warm-starting, tuning or control rate, because both optimizers see the same problem
# at the same state. mppi_warm:=0 keeps each probe an independent solve from that guess.
run_one B8-PROBE map4 navigation_STLMPC_vary_v "use_mppi:=0 mppi_probe:=1 mppi_warm:=1 mppi_lambda:=-1"

# B4b -- the two final baseline rows.
#  * FGM in the DYNAMIC encounter (map2, braking adversary), matching B5-R1/B6-N1.
#    The adversary is not part of the occupancy grid, so a purely scan-based gap
#    follower has no way to perceive it at all: this row shows what the detection and
#    prediction branch of Section IV provides, which the heuristic cannot express.
#  * MPPI at a matched COMPUTE budget. The 2048-rollout configuration takes 137.6 ms
#    per step against SQP's 36 ms, so it also runs below the 10 Hz control rate; at
#    mppi_K=64 (512 rollouts, ~34 ms) it is inside SQP's budget and controls at rate,
#    removing that confound from the closed-loop comparison.
run_one B4-FGM-dyn  map2 navigation_FGM "adv_enable:=1 adv_maneuver:=brake"
run_one B4-MPPI-tm  map4 navigation_STLMPC_vary_v "use_mppi:=1 mppi_warm:=1 mppi_lambda:=-1 mppi_K:=64"

# B5 -- robustness. Dynamic adversary (via TF->EKF) on the open map (map2).
run_one B5-R1 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=brake"
run_one B5-R2 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=swerve"
run_one B5-R3 map2 navigation_STLMPC "adv_enable:=1 adv2_enable:=1 adv_maneuver:=straight"
for s in 0.01 0.03 0.05;      do run_one "B5-R4-$s" map1 navigation_STLMPC "scan_std_dev:=$s beam_dropout_prob:=0.05"; done

# B6 -- dynamic-obstacle PREDICTION BRANCH ablation (the third modular ablation the
# reviewers asked for, alongside B1 sequential lines and B2 velocity constraints).
# Identical to B5-R1/R2/R3 but with use_neural_net:=0, which disables the TF->EKF
# detection/prediction branch that augments the scan with the adversary's projected
# path. The adversary is not in the occupancy map, so with the branch off the ego is
# effectively blind to it; the effect is therefore measured by minTTC / min_rel_dist
# from the logged relative pose (compute_ttc.py), not by d_min against the map.
run_one B6-N1 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=brake use_neural_net:=0"
run_one B6-N2 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=swerve use_neural_net:=0"
run_one B6-N3 map2 navigation_STLMPC "adv_enable:=1 adv2_enable:=1 adv_maneuver:=straight use_neural_net:=0"

# B5 generalization -- unseen maps not used in the ablations (map3, and the map5 fork).
run_one B5-GEN-m3 map3 navigation_STLMPC_vary_v ""
run_one B5-GEN-m5 map5 navigation_STLMPC_vary_v ""

echo "Campaign complete. Logs in $OUT"
echo "Next: python3 scripts/aggregate_runs.py $OUT --latex --collision-radius 0.15"
