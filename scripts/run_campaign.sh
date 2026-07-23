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
    rosnode kill /navigation_STLMPC >/dev/null 2>&1
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
run_one B4-FGM  map1 navigation_FGM ""
run_one B4-FGM  map2 navigation_FGM ""
run_one B4-FGM  map4 navigation_FGM ""
run_one B4-MPPI map4 navigation_STLMPC_vary_v "use_mppi:=1"
run_one B4-SQP  map4 navigation_STLMPC_vary_v "use_mppi:=0"   # matched SQP reference

# B5 -- robustness. Dynamic adversary (via TF->EKF) on the open map (map2).
run_one B5-R1 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=brake"
run_one B5-R2 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=swerve"
run_one B5-R3 map2 navigation_STLMPC "adv_enable:=1 adv2_enable:=1 adv_maneuver:=straight"
for s in 0.01 0.03 0.05;      do run_one "B5-R4-$s" map1 navigation_STLMPC "scan_std_dev:=$s beam_dropout_prob:=0.05"; done

# B5 generalization -- unseen maps not used in the ablations (map3, and the map5 fork).
run_one B5-GEN-m3 map3 navigation_STLMPC_vary_v ""
run_one B5-GEN-m5 map5 navigation_STLMPC_vary_v ""

echo "Campaign complete. Logs in $OUT"
echo "Next: python3 scripts/aggregate_runs.py $OUT --latex --collision-radius 0.15"
