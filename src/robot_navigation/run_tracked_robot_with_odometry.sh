#!/bin/bash

# Set display for GUI applications
export DISPLAY=:1

# Check if virtual X server is needed and available
if ! xdpyinfo -display :1 &> /dev/null; then
    echo "Starting Xvfb display server"
    Xvfb :1 -screen 0 1024x768x24 &
    XVFB_PID=$!
    trap "kill $XVFB_PID" EXIT
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Workspace directory
WORKSPACE_DIR="/home/x4/ros2_ws_arduino"
cd ${WORKSPACE_DIR}

# Ensure directories exist
mkdir -p ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes

# Copy URDF and mesh files to installation directory
echo "Copying URDF files to installation directory..."
cp -f ${WORKSPACE_DIR}/src/robot_navigation/urdf/tracked_robot.urdf.xacro ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/
cp -f ${WORKSPACE_DIR}/src/robot_navigation/urdf/meshes/simple_track.dae ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes/

# Build the package
echo "Building robot_navigation package..."
colcon build --packages-select robot_navigation

# Source the workspace
source ${WORKSPACE_DIR}/install/setup.bash

# Make sure the track mesh exists
echo "Checking mesh files..."
if [ ! -f "${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes/simple_track.dae" ]; then
    echo "Missing track mesh file. Creating a simple cylinder mesh..."
    # Create a simple dae file as a placeholder
    mkdir -p "${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes"
    cat > "${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/meshes/simple_track.dae" << EOL
<?xml version="1.0" encoding="utf-8"?>
<COLLADA version="1.4.1" xmlns="http://www.collada.org/2005/11/COLLADASchema">
  <library_geometries>
    <geometry id="track">
      <mesh>
        <source id="track-positions">
          <float_array id="track-positions-array" count="24">-1 -1 -1 1 -1 -1 -1 1 -1 1 1 -1 -1 -1 1 1 -1 1 -1 1 1 1 1 1</float_array>
          <technique_common>
            <accessor source="#track-positions-array" count="8" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="track-vertices">
          <input semantic="POSITION" source="#track-positions"/>
        </vertices>
        <polylist count="12">
          <input semantic="VERTEX" source="#track-vertices" offset="0"/>
          <vcount>3 3 3 3 3 3 3 3 3 3 3 3</vcount>
          <p>0 1 2 2 1 3 4 6 5 6 7 5 0 4 1 1 4 5 2 3 6 3 7 6 0 2 4 2 6 4 1 5 3 3 5 7</p>
        </polylist>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene">
      <node id="Track">
        <instance_geometry url="#track"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
EOL
fi

# Process the URDF file with xacro
echo "Processing URDF file with xacro..."
# Create a temporary URDF file - remove the Gazebo plugin parts since they cause issues
TMP_XACRO=$(mktemp)
grep -v -E "gazebo|libgazebo|Gazebo/" ${WORKSPACE_DIR}/install/robot_navigation/share/robot_navigation/urdf/tracked_robot.urdf.xacro > $TMP_XACRO
URDF_OUTPUT=$(ros2 run xacro xacro $TMP_XACRO)
rm $TMP_XACRO

# Create the final URDF file
URDF_FILE=$(mktemp)
echo "$URDF_OUTPUT" > $URDF_FILE

# Publish the robot description parameter
echo "Publishing robot_description parameter..."
ros2 param set /robot_description robot_description "$URDF_OUTPUT" --no-daemon &
ROSPARAM_PID=$!
sleep 2

# Launch the robot state publisher
echo "Launching robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$URDF_OUTPUT" \
    -p use_sim_time:=false &
RSP_PID=$!

# Launch the odometry publisher
echo "Launching simulated odometry..."
ros2 run robot_navigation simulated_odometry --ros-args -p update_rate:=30.0 &
ODOM_PID=$!

# Launch the joint state publisher
echo "Launching joint state publisher..."
ros2 run joint_state_publisher joint_state_publisher \
    --ros-args -p rate:=30 -p use_sim_time:=false &
JSP_PID=$!

# Launch the joint state publisher GUI
echo "Launching joint state publisher GUI..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSPGUI_PID=$!

# Launch RViz with a default configuration
echo "Launching RViz..."
ros2 run rviz2 rviz2 &
RVIZ_PID=$!

echo "All nodes are running. Press Ctrl+C to stop."
echo "Robot model is visible in RViz."

# Set up cleanup on exit
trap "kill $ROSPARAM_PID $RSP_PID $ODOM_PID $JSP_PID $JSPGUI_PID $RVIZ_PID; rm -f $URDF_FILE" EXIT SIGINT SIGTERM

# Wait for user to press Ctrl+C
wait
