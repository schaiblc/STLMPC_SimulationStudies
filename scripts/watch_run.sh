#!/usr/bin/env bash
# watch_run.sh -- launch ONE campaign config with rviz so it can be watched.
#
#   bash scripts/watch_run.sh mppi     # MPPI on map4 (0% completion, 0.27 m clearance)
#   bash scripts/watch_run.sh fork     # map5 fork: goal-free branch selection
#   bash scripts/watch_run.sh fgm1     # FGM wandering on map1
#   bash scripts/watch_run.sh fgm2     # FGM wandering on map2
#   bash scripts/watch_run.sh sqp      # STLMPC-SQP on map4, for side-by-side with mppi
#
# Optional 2nd arg = seed (reproduces that seed's randomized start, as the sweep did):
#   bash scripts/watch_run.sh mppi 3
#
# The green sphere + ring in rviz is the course goal. Watch the console for
# "COURSE COMPLETE ... in T s" or "[completion] TIMEOUT".
set -u
CASE=${1:-mppi}; SEED=${2:-}
case "$CASE" in
  mppi) MAP=map4; PLN=navigation_STLMPC_vary_v; EXTRA="use_mppi:=1";;
  sqp)  MAP=map4; PLN=navigation_STLMPC_vary_v; EXTRA="use_mppi:=0";;
  fork) MAP=map5; PLN=navigation_STLMPC_vary_v; EXTRA="";;
  fgm1) MAP=map1; PLN=navigation_FGM;           EXTRA="";;
  fgm2) MAP=map2; PLN=navigation_FGM;           EXTRA="";;
  *) echo "unknown case '$CASE' (mppi|sqp|fork|fgm1|fgm2)"; exit 1;;
esac

# Reproduce a specific sweep seed's start jitter, matching run_campaign.sh's formula.
J=""
if [ -n "$SEED" ]; then
  jit(){ awk -v s="$1" -v k="$2" 'BEGIN{srand(s*97+k*7919); print 2*rand()-1}'; }
  dx=$(awk -v j="$(jit "$SEED" 1)" 'BEGIN{print 0.3*j}')
  dy=$(awk -v j="$(jit "$SEED" 2)" 'BEGIN{print 0.3*j}')
  dt=$(awk -v j="$(jit "$SEED" 3)" 'BEGIN{print 0.1745*j}')
  J="ego_jitter_x:=$dx ego_jitter_y:=$dy ego_jitter_theta:=$dt"
  echo ">>> reproducing seed $SEED jitter=($dx,$dy,$dt)"
fi

echo ">>> $CASE: map=$MAP planner=$PLN $EXTRA"
exec roslaunch f1tenth_simulator campaign.launch gui:=true \
     map_name:="$MAP" planner:="$PLN" \
     auto_nav:=1 auto_nav_delay:=3.0 \
     shutdown_on_goal:=1 run_timeout:=60 enable_logging:=0 \
     $EXTRA $J
