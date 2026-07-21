#!/usr/bin/env bash
# run_campaign.sh -- batch driver for the STLMPC revision simulation campaign.
#
# For each configuration x map x seed it: (1) computes a deterministic randomized
# start pose (+/-0.3 m, +/-10 deg) from the seed, (2) launches campaign.launch
# headless with the configuration's param overrides and a per-run log file,
# (3) resets the ego pose via /initialpose, (4) lets it run for RUN_SECONDS,
# (5) kills the launch. Log files are named <CONFIG>_map<M>_seed<NN>.csv so that
# scripts/aggregate_runs.py can recover config/map/seed.
#
# This is a TEMPLATE: verify the roslaunch invocation and the /initialpose reset
# against your setup before a full sweep (do a single dry run first). The
# config->param mapping below is the authoritative, non-obvious part.
#
# Requires a sourced ROS Noetic workspace (roslaunch, rostopic on PATH).
set -u

PKG=f1tenth_simulator
OUT=${OUT:-$HOME/stlmpc_logs}          # where CSVs are written
SEEDS=${SEEDS:-10}                      # seeds per configuration
RUN_SECONDS=${RUN_SECONDS:-60}          # sim seconds per run (kill after)
mkdir -p "$OUT"

MAP1=$(rospack find $PKG)/maps/map1.yaml   # static
MAP2=$(rospack find $PKG)/maps/map2.yaml   # dynamic
MAP3=$(rospack find $PKG)/maps/map3.yaml   # racing

# Per-map nominal start pose (x y theta). Adjust to a valid free cell on each map.
declare -A START_X=( [map1]=0.0 [map2]=0.0 [map3]=0.0 )
declare -A START_Y=( [map1]=0.0 [map2]=0.0 [map3]=0.0 )
declare -A START_T=( [map1]=0.0 [map2]=0.0 [map3]=0.0 )
declare -A MAPFILE=( [map1]=$MAP1 [map2]=$MAP2 [map3]=$MAP3 )

# deterministic pseudo-random jitter in [-1,1] from (seed, salt)
jit() { awk -v s="$1" -v k="$2" 'BEGIN{srand(s*97+k*7919); print 2*rand()-1}'; }

# run_one CONFIG MAPKEY PLANNER "extra roslaunch args..."
run_one() {
  local cfg="$1" mapk="$2" planner="$3"; shift 3
  local extra="$*"
  for ((sd=0; sd<SEEDS; sd++)); do
    local seed=$(printf "%02d" "$sd")
    local log="$OUT/${cfg}_${mapk}_seed${seed}.csv"
    local dx dy dt
    dx=$(awk -v j="$(jit "$sd" 1)" 'BEGIN{print 0.3*j}')
    dy=$(awk -v j="$(jit "$sd" 2)" 'BEGIN{print 0.3*j}')
    dt=$(awk -v j="$(jit "$sd" 3)" 'BEGIN{print 0.1745*j}')  # 10 deg in rad
    local px py pt
    px=$(awk -v a="${START_X[$mapk]}" -v b="$dx" 'BEGIN{print a+b}')
    py=$(awk -v a="${START_Y[$mapk]}" -v b="$dy" 'BEGIN{print a+b}')
    pt=$(awk -v a="${START_T[$mapk]}" -v b="$dt" 'BEGIN{print a+b}')

    echo ">>> $cfg $mapk seed=$seed  start=($px,$py,$pt)"
    roslaunch $PKG campaign.launch \
        map:="${MAPFILE[$mapk]}" planner:="$planner" \
        log_file:="$log" enable_logging:=1 $extra \
        >/dev/null 2>&1 &
    local lpid=$!
    sleep 3   # let nodes come up

    # reset ego to the randomized start pose
    rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
      "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $px, y: $py, z: 0.0}, \
        orientation: {z: $(awk -v t=$pt 'BEGIN{print sin(t/2)}'), \
                      w: $(awk -v t=$pt 'BEGIN{print cos(t/2)}')}}}}" \
      >/dev/null 2>&1
    sleep 1

    # enable navigation mode (behavior_controller toggles the nav mux on nav_key_char).
    # Without this the planner idles (nav_active=0) and the vehicle never moves.
    rostopic pub -1 /key std_msgs/String "data: 'n'" >/dev/null 2>&1

    sleep "$RUN_SECONDS"
    kill -INT $lpid 2>/dev/null
    wait $lpid 2>/dev/null
    sleep 2
  done
}

# ----------------------------------------------------------------------------
# B1 -- sequential-vs-single tracking line (fixed 1.6 s horizon), static map #1
run_one B1-A1 map1 navigation_STLMPC "nMPC:=1 kMPC:=16"
run_one B1-A2 map1 navigation_STLMPC "nMPC:=2 kMPC:=8"
run_one B1-A3 map1 navigation_STLMPC "nMPC:=4 kMPC:=4"

# B2 -- velocity-constraint ablation, racing map #3, variable-velocity node
run_one B2-V1 map3 navigation_STLMPC_vary_v "enable_gvsteer:=1 enable_gvobs:=1 hard_clamp_v:=0"
run_one B2-V2 map3 navigation_STLMPC_vary_v "enable_gvsteer:=0 enable_gvobs:=1 hard_clamp_v:=0"
run_one B2-V3 map3 navigation_STLMPC_vary_v "enable_gvsteer:=1 enable_gvobs:=0 hard_clamp_v:=0"
run_one B2-V4 map3 navigation_STLMPC_vary_v "enable_gvsteer:=0 enable_gvobs:=0 hard_clamp_v:=1"

# B3 -- one-at-a-time sensitivity (5 values/param). Config token = B3-<param>-<value>.
for v in 10 17 30 52 90;      do run_one "B3-lambda_dd-$v" map1 navigation_STLMPC_vary_v "d_dot_factor_STLMPC_vary_v:=$v"; done
for v in 0.33 0.58 1 1.7 3;   do run_one "B3-lambda_v-$v"  map3 navigation_STLMPC_vary_v "vel_factor_STLMPC_vary_v:=$v"; done
for v in 0.75 1 1.25 1.6 2;   do run_one "B3-d_safe-$v"    map3 navigation_STLMPC_vary_v "safe_distance:=$v"; done
for v in 3.3 5.8 10 17 30;    do run_one "B3-beta_v-$v"    map3 navigation_STLMPC_vary_v "vel_beta:=$v"; done
for v in 0.17 0.29 0.5 0.87 1.5; do run_one "B3-alpha_v-$v" map3 navigation_STLMPC_vary_v "stop_distance_decay:=$v"; done
for v in 67 115 200 350 600;  do run_one "B3-s_theta-$v"   map3 navigation_STLMPC_vary_v "theta_band_smooth:=$v"; done

# B4 -- baselines. FGM (gap heuristic) on maps #1-#3; MPPI (sampling optimizer on
# the SAME STLMPC objective+constraints) on the racing map vs variable-velocity SQP.
run_one B4-FGM  map1 navigation_FGM ""
run_one B4-FGM  map2 navigation_FGM ""
run_one B4-FGM  map3 navigation_FGM ""
run_one B4-MPPI map3 navigation_STLMPC_vary_v "use_mppi:=1"
run_one B4-SQP  map3 navigation_STLMPC_vary_v "use_mppi:=0"   # matched SQP reference

# B5 -- robustness (dynamic node ingests the adversary via TF->EKF)
run_one B5-R1 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=brake"
run_one B5-R2 map2 navigation_STLMPC "adv_enable:=1 adv_maneuver:=swerve"
run_one B5-R3 map2 navigation_STLMPC "adv_enable:=1 adv2_enable:=1 adv_maneuver:=straight"
for s in 0.01 0.03 0.05;      do run_one "B5-R4-$s" map1 navigation_STLMPC "scan_std_dev:=$s beam_dropout_prob:=0.05"; done

echo "Campaign complete. Logs in $OUT"
echo "Next: python3 scripts/aggregate_runs.py $OUT --latex --collision-radius 0.15"
