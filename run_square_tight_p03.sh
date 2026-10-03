#!/usr/bin/env bash
set -eo pipefail

workspace=/home/hoang/Paper1/Paper1-main
site_packages="$workspace/install/amr_control/lib/python3.10/site-packages"

# A terminal opened from Paper1-cu/src/amr_control puts that legacy package at
# the front of Python's search path.  Move to the intended workspace and build
# a clean ROS environment before resolving the controller entry point.
cd "$workspace"
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u
export MPLCONFIGDIR=/tmp/paper1-matplotlib
mkdir -p "$MPLCONFIGDIR"

square_source="$(${PYTHON:-python3} -c \
    'import amr_control.bsmc_square as module; print(module.__file__)')"
capture_source="$(${PYTHON:-python3} -c \
    'import amr_control.embedded_paper_capture as module; print(module.__file__)')"

case "$square_source" in
    "$site_packages"/*) ;;
    *)
        echo "REFUSING TO MOVE: unexpected bsmc_square: $square_source" >&2
        exit 2
        ;;
esac
case "$capture_source" in
    "$site_packages"/*) ;;
    *)
        echo "REFUSING TO MOVE: unexpected capture module: $capture_source" >&2
        exit 2
        ;;
esac

echo "Verified controller: $square_source"
echo "Verified capture:    $capture_source"
echo "Pilot: ki_y=0.12, integral seed=0.25, limit=0.30, one lap"

if [[ "${1:-}" == "--check" ]]; then
    echo "Check only: robot command was not started."
    exit 0
fi
if (( $# != 0 )); then
    echo "Usage: $0 [--check]" >&2
    exit 2
fi

exec ros2 run amr_control bsmc_square --ros-args \
    -p square_profile:=2m \
    -p paper_laps:=1 \
    -p ki_y_straight:=0.12 \
    -p initial_ey_integral:=0.25 \
    -p ey_integral_limit:=0.30 \
    -p paper_run_id:=square_bsmc_tight_iy_p03
