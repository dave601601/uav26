#!/bin/bash
# dev.sh — one-command driver for the uav26 sim workspace (run on the
# WSL host from the repo root or anywhere).
#
#   scripts/dev.sh gui [seed]             Gazebo GUI + line_tracer + both
#                                         detection overlays (downward and
#                                         sideways lookahead).
#                                         Ctrl+C stops everything.
#   scripts/dev.sh view [down|side]       rqt_image_view on one overlay
#                                         (default down). Run alongside
#                                         gui or mission.
#   scripts/dev.sh mission [run] [dur] [launch_args]
#                                         Headless mission via
#                                         run_mission.sh (default 900 s),
#                                         then prints the FSM summary.
#                                         launch_args go to the tracer
#                                         launch — use params_file:=... to
#                                         A/B a variant parameter set.
#   scripts/dev.sh build                  (Re)build if install/ is gone
#                                         (container recreation drops it).
#
# Handles the traps discovered the hard way:
# - Docker Desktop resolves absolute bind-mount paths inside its own
#   utility VM, so the WSLg X socket must be bridged through /mnt/wsl
#   (shared across every WSL distro). gui mode creates the bridge with
#   sudo if missing and recreates the container if it predates it.
# - /workspace/install lives in the container layer, not a mount —
#   any recreate needs a rebuild (fast: ./build cache is a mount).
# - Teardown must SIGINT the launches and then kill fc_sim_node /
#   parameter_bridge BY NAME, or orphans fight the next run. The sweeper
#   itself must be fed on stdin — see the sweep() comment.
set -e
cd "$(dirname "$0")/.."

DC="docker compose"
EXEC="$DC exec -T uav-aruco"

ensure_x_bridge() {
  # Userspace relay (scripts/x11_bridge.py): repo-local ./.x11-bridge/X0
  # -> real WSLg socket. Needed because Docker Desktop resolves
  # absolute mount paths in its own VM (ghost Xwayland). No sudo.
  mkdir -p .x11-bridge
  if [ -f .x11-bridge/pid ] && kill -0 "$(cat .x11-bridge/pid)" 2>/dev/null \
      && [ -S .x11-bridge/X0 ]; then
    return
  fi
  echo "[dev] starting userspace X11 bridge"
  nohup python3 scripts/x11_bridge.py >/dev/null 2>&1 &
  sleep 1
  [ -S .x11-bridge/X0 ] || { echo "[dev] X11 bridge failed to start (see .x11-bridge/log)"; exit 1; }
}

ensure_container() {
  $DC up -d uav-aruco
}

ensure_build() {
  if ! $EXEC test -f /workspace/install/setup.bash 2>/dev/null; then
    echo "[dev] install/ missing — colcon build (fast: build cache is mounted)"
    $EXEC bash -lc "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --packages-ignore realsense2_camera realsense2_camera_msgs"
  fi
}

view_topic() {
  $EXEC bash -lc "source /opt/ros/jazzy/setup.bash && \
    ros2 run rqt_image_view rqt_image_view $1"
}

# Piped through `bash -s`, never `bash -c "<text>"`. `pkill -f` matches a
# full command line, and with -c the patterns are IN the sweeping shell's
# own command line — so `pkill -9 -f 'gz sim'` SIGKILLs the sweeper before
# it ever reaches fc_sim_node / parameter_bridge / line_tracer_node.
# Measured: `bash -c "pkill -9 -f 'gz sim'; echo x"` exits 137 and prints
# nothing, and planted decoys survive. That silently disabled the whole
# -9 pass — the by-name kills this contract exists for. run_mission.sh
# was never affected because it has always been fed on stdin.
sweep() {
  $EXEC bash -s <<'SWEEP' || true
pkill -INT -f 'ros2 launch' 2>/dev/null
sleep 3
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f fc_sim_node 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f line_tracer_node 2>/dev/null
pkill -9 -f rqt_image_view 2>/dev/null
pkill -9 -f 'ros2 launch' 2>/dev/null
true
SWEEP
}

case "${1:-gui}" in
  build)
    ensure_container
    ensure_build
    ;;

  gui)
    SEED=${2:-42}
    ensure_x_bridge
    ensure_container
    if ! $EXEC test -S /tmp/.X11-unix/X0 2>/dev/null; then
      echo "[dev] container predates the X bridge — recreating"
      $DC up -d --force-recreate uav-aruco
    fi
    ensure_build
    sweep
    trap 'echo; echo "[dev] stopping..."; sweep; exit 0' INT TERM
    echo "[dev] Gazebo GUI up (seed=$SEED); line_tracer follows in 8 s."
    echo "[dev] then two overlay windows: downward + sideways lookahead."
    echo "[dev] Ctrl+C stops sim + tracer + viewers and sweeps strays."
    $EXEC bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch world sim.launch.py marker_seed:=$SEED" &
    sleep 8
    $EXEC bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch line_tracer line_tracer.launch.py" &
    # rqt_image_view binds the topic argument against what is advertised
    # when it starts, so it has to come up after the tracer. The lookahead
    # overlay now publishes in every FSM state, so its window is live
    # immediately instead of black until the sweep begins.
    sleep 6
    view_topic /line_tracer/debug_image &
    view_topic /line_tracer/lookahead_debug_image &
    wait
    ;;

  view)
    ensure_x_bridge
    ensure_container
    case "${2:-down}" in
      side|lookahead)   TOPIC=/line_tracer/lookahead_debug_image ;;
      down|downward|"") TOPIC=/line_tracer/debug_image ;;
      *) echo "usage: $0 view [down|side]" >&2; exit 1 ;;
    esac
    echo "[dev] rqt_image_view on $TOPIC (close the window or Ctrl+C to exit)"
    view_topic "$TOPIC"
    ;;

  mission)
    RUN=${2:-r$(date +%H%M%S)}
    DUR=${3:-900}
    EXTRA=${4:-}
    ensure_container
    ensure_build
    echo "[dev] headless mission $RUN (${DUR}s, seed 42) ${EXTRA:+[$EXTRA]}"
    $EXEC bash -s "$RUN" "$DUR" "$EXTRA" < scripts/run_mission.sh
    LOG="build/sweep_logs/mission/${RUN}_tracer.log"
    echo "[dev] ---- FSM events ----"
    grep -E ">> (FSM|RECORD)" "$LOG" | sed 's/.*line_tracer_node\]: //' || true
    echo "[dev] ---- final pose ----"
    tail -2 "$LOG" | grep -o "xy=([^)]*) alt=[0-9.]*" | tail -1 || true
    echo "[dev] ---- gz aborts: $(grep -c Aborted "build/sweep_logs/mission/${RUN}_sim.log" 2>/dev/null || echo 0) ----"
    ;;

  *)
    echo "usage: $0 {gui [seed] | view [down|side] | mission [run] [dur] [launch_args] | build}"
    exit 1
    ;;
esac
