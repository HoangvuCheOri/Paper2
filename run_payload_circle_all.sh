#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/hoang/Paper1/Paper1-main"
SESSION_ID="${1:-payload_circle_set01}"
BACKGROUND_LOG_DIR="$WORKSPACE/paper_runs/payload_circle/$SESSION_ID/background_logs"
declare -a STARTED_PIDS=()

cleanup() {
    if ((${#STARTED_PIDS[@]})); then
        echo "Stopping background nodes started by this script..."
        kill -INT "${STARTED_PIDS[@]}" 2>/dev/null || true
        wait "${STARTED_PIDS[@]}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_control --symlink-install
source "$WORKSPACE/install/setup.bash"
if [[ -f /tmp/paper1_camera.env ]]; then
    source /tmp/paper1_camera.env
fi
mkdir -p "$BACKGROUND_LOG_DIR"

node_exists() {
    ros2 node list 2>/dev/null | grep -Fxq "$1"
}

start_if_missing() {
    local node_name="$1"
    local log_name="$2"
    shift 2
    if node_exists "$node_name"; then
        echo "Using existing node: $node_name"
        return
    fi
    echo "Starting $node_name (log: $BACKGROUND_LOG_DIR/$log_name.log)"
    "$@" >"$BACKGROUND_LOG_DIR/$log_name.log" 2>&1 &
    STARTED_PIDS+=("$!")
}

wait_for_topic() {
    local topic="$1"
    echo "Waiting for one message on $topic ..."
    if ! timeout 45 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        echo "ERROR: no message received from $topic within 45 seconds." >&2
        echo "Check logs in $BACKGROUND_LOG_DIR" >&2
        exit 1
    fi
}

start_if_missing /robot_serial_bridge robot_serial_bridge \
    ros2 run amr_control robot_serial_bridge
start_if_missing /state_bridge_node state_bridge \
    ros2 run amr_control state_bridge
start_if_missing /custom_ekf_node custom_ekf_node \
    ros2 run amr_control custom_ekf_node
start_if_missing /pose_estimation_publisher camera_circle_square \
    ros2 run amr_control camera_circle_square

wait_for_topic /odom_camera
wait_for_topic /odometry/filtered
wait_for_topic /espnow_link

echo
echo "All background topics are ready."
echo "The runner will pause before every run so you can change the payload."
echo "Keep the 5 cm offset toward the FRONT of the robot."
echo

ros2 run amr_control payload_circle_experiment \
    --execute \
    --session-id "$SESSION_ID" \
    --case all \
    --controller all \
    --repeat 1 \
    --laps 3 \
    --angular-speed 0.108 \
    --base-mass 1.55 \
    --offset-direction forward

echo
echo "All 21 runs completed. Output:"
echo "$WORKSPACE/paper_runs/payload_circle/$SESSION_ID"
