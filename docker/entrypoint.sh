#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ARGOS_PLUGIN_PATH=/ros2_ws/install/argos3_ros_bridge/lib:$ARGOS_PLUGIN_PATH
export LD_PRELOAD=$(find /usr/local/lib/argos3 -name "*.so" | tr '\n' ':')
exec "$@"