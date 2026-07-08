#!/bin/bash
# Headless end-to-end mission run for the uav-aruco container.
#
# Usage (from the repo root, on the host):
#   docker compose exec -T uav-aruco bash -s r53 < scripts/run_mission.sh
#
# Logs land in build/sweep_logs/mission/<run>_{sim,tracer}.log (the
# build/ directory is bind-mounted, so they are visible on the host).
#
# CLEANUP CONTRACT
# ----------------
# pkill -9 on 'ros2 launch' orphans its children: launch propagates
# SIGTERM to child nodes only on a *catchable* shutdown. An orphaned
# fc_sim_node auto-arms again the moment the NEXT run's gz publishes
# /clock + /imu (topics are absolute), so two flight controllers fight
# for one drone — this poisoned runs r42..r51. Therefore:
#   1. sweep strays BEFORE starting (fail loud if any survive),
#   2. tear down with SIGINT to the launch processes first,
#   3. only then pkill -9 the stragglers, INCLUDING fc_sim_node and
#      parameter_bridge by name.
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
set -u   # after sourcing: ROS setup scripts reference unbound vars

RUN=${1:-r$(date +%H%M%S)}
LOGDIR=/workspace/build/sweep_logs/mission
mkdir -p "$LOGDIR"

sweep() {
  pkill -9 -f 'gz sim' 2>/dev/null
  pkill -9 -f fc_sim_node 2>/dev/null
  pkill -9 -f parameter_bridge 2>/dev/null
  pkill -9 -f line_tracer_node 2>/dev/null
  pkill -9 -f 'ros2 launch' 2>/dev/null
}

sweep; sleep 1
STRAYS=$(ps aux | grep -E 'fc_sim_node|parameter_bridge|gz sim' | grep -v grep | wc -l)
echo "pre-run stray processes: $STRAYS"
if [ "$STRAYS" -ne 0 ]; then
  echo "ABORT: stray sim processes survived the sweep" >&2
  exit 1
fi

ros2 launch world sim.launch.py headless:=true marker_seed:=42 \
  > "$LOGDIR/${RUN}_sim.log" 2>&1 &
SIM_PID=$!
sleep 8
stdbuf -oL -eL ros2 launch line_tracer line_tracer.launch.py \
  > "$LOGDIR/${RUN}_tracer.log" 2>&1 &
TRACER_PID=$!

sleep 150
kill -INT "$TRACER_PID" 2>/dev/null
sleep 3
kill -INT "$SIM_PID" 2>/dev/null
sleep 5
sweep
echo "MISSION_DONE ${RUN}"
