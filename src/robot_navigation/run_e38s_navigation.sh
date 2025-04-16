#!/bin/bash

# Run navigation using high-resolution E38S encoders for odometry
# This script launches the encoder odometry node with Nav2 integration

# Default values
SERIAL_PORT="/dev/ttyACM0"
WHEEL_RADIUS="0.0825"
BASE_WIDTH="0.17"
TICKS_PER_REV="4000.0"
MAX_LINEAR_SPEED="0.5"
MAX_ANGULAR_SPEED="2.0"
PUBLISH_RATE="20.0"
USE_RVIZ="true"
MAP_FILE="simple_map.yaml"

# Parse command line arguments
while [ $# -gt 0 ]; do
  case "$1" in
    --serial-port=*)
      SERIAL_PORT="${1#*=}"
      ;;
    --wheel-radius=*)
      WHEEL_RADIUS="${1#*=}"
      ;;
    --base-width=*)
      BASE_WIDTH="${1#*=}"
      ;;
    --ticks-per-rev=*)
      TICKS_PER_REV="${1#*=}"
      ;;
    --max-linear-speed=*)
      MAX_LINEAR_SPEED="${1#*=}"
      ;;
    --max-angular-speed=*)
      MAX_ANGULAR_SPEED="${1#*=}"
      ;;
    --publish-rate=*)
      PUBLISH_RATE="${1#*=}"
      ;;
    --no-rviz)
      USE_RVIZ="false"
      ;;
    --map=*)
      MAP_FILE="${1#*=}"
      ;;
    --help)
      echo "Usage: $0 [options]"
      echo ""
      echo "Options:"
      echo "  --serial-port=PORT     Arduino serial port (default: /dev/ttyACM0)"
      echo "  --wheel-radius=VAL     Wheel radius in meters (default: 0.0825)"
      echo "  --base-width=VAL       Distance between wheels in meters (default: 0.17)"
      echo "  --ticks-per-rev=VAL    Encoder ticks per revolution (default: 4000.0 for E38S encoders)"
      echo "  --max-linear-speed=VAL Maximum linear speed in m/s (default: 0.5)"
      echo "  --max-angular-speed=VAL Maximum angular speed in rad/s (default: 2.0)"
      echo "  --publish-rate=VAL     Odometry publish rate in Hz (default: 20.0)"
      echo "  --no-rviz              Disable RViz visualization"
      echo "  --map=FILENAME         Map file to use (default: simple_map.yaml)"
      echo "  --help                 Display this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage information."
      exit 1
      ;;
  esac
  shift
done

# Make sure we're in the right directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_WS="$WORKSPACE_DIR"

# Source the workspace
source "$ROS_WS/install/setup.bash"

# If we don't have a full path for the map, assume it's in the maps directory
if [[ "$MAP_FILE" != /* ]]; then
    MAP_FILE="$SCRIPT_DIR/maps/$MAP_FILE"
fi

echo "Using map: $MAP_FILE"
echo "Using E38S high-resolution encoders with $TICKS_PER_REV ticks per revolution"
echo "Serial port: $SERIAL_PORT"

# Launch navigation with encoder odometry
if [ "$USE_RVIZ" = "true" ]; then
    # Launch with RViz visualization
    ros2 launch robot_navigation navigation_rviz.launch.py \
        use_sim_time:=false \
        map:=$MAP_FILE \
        encoder_odometry:=true \
        serial_port:=$SERIAL_PORT \
        wheel_radius:=$WHEEL_RADIUS \
        base_width:=$BASE_WIDTH \
        ticks_per_rev:=$TICKS_PER_REV \
        max_linear_speed:=$MAX_LINEAR_SPEED \
        max_angular_speed:=$MAX_ANGULAR_SPEED \
        publish_rate:=$PUBLISH_RATE
else
    # Launch without RViz
    ros2 launch robot_navigation navigation.launch.py \
        use_sim_time:=false \
        map:=$MAP_FILE \
        encoder_odometry:=true \
        serial_port:=$SERIAL_PORT \
        wheel_radius:=$WHEEL_RADIUS \
        base_width:=$BASE_WIDTH \
        ticks_per_rev:=$TICKS_PER_REV \
        max_linear_speed:=$MAX_LINEAR_SPEED \
        max_angular_speed:=$MAX_ANGULAR_SPEED \
        publish_rate:=$PUBLISH_RATE
fi
