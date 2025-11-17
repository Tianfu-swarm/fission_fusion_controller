#!/bin/bash
ros2 daemon stop
ros2 daemon start

sudo find /dev/shm -maxdepth 1 -type f \
  \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \) \
  -delete
# Define a function to clean up processes when the script is terminated
cleanup() {
    echo "Stopping ARGoS3, ROS 2, rosbag, and RViz..."
    pkill -f argos3 # Kill ARGoS3 process
    pkill -f ros2   # Kill all ROS 2 processes
    pkill -f rviz2  # Kill RViz process
    pkill -f fission_fusion_controller
    exit 0
}

# Trap SIGINT (Ctrl+C) and call cleanup function
trap cleanup SIGINT

source ~/fission_fusion_controller_ws/install/setup.bash
# Export necessary environment variables
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:../../../install/argos3_ros_bridge/lib
export ARGOS_PLUGIN_PATH=../../../install/argos3_ros_bridge/lib

ROS2_CORES="0-9"     # 分配核 0 到 7 给 ROS 2 节点
ARGOS_CORE="10-13"       # 分配核 8 给 ARGoS3

# Disable Fast DDS shared memory transport to avoid /dev/shm permission issue
export RMW_FASTRTPS_USE_SHM=OFF

timestamp=$(date +%s)
domain_id=$((timestamp % 233))
export ROS_DOMAIN_ID=$domain_id
# export ROS_DOMAIN_ID=100
echo "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"

timestamp_result=$(date +"%Y%m%d%H%M")
results_path="../../data/result_${timestamp_result}"

# Run the ROS 2 launch command in the background
taskset -c $ROS2_CORES \
ros2 launch fission_fusion_controller run.launch.py numbers:=12.0 \
                                                    desired_subgroup_size:=14.0 \
                                                    follow_range:=5.0 \
                                                    subgroup_size_sigma:=0.0 \
                                                    groupsize_tolerance:=0.0 \
                                                    K:=1000 \
                                                    early_converge_window:=4 \
                                                    isModelworks:=false \
                                                    isMinCommunication:=true \
                                                    isConCommunication:=true \
                                                    use_rviz:=true \
                                                    use_sim_time:=true \
                                                    results_file_path:=${results_path}&
ROS2_PID=$! # Save the process ID (PID) of ros2 launch

# Wait for ROS 2 nodes to initialize
sleep 1 # Increase sleep time if necessary

# Run ARGoS3 with the specified configuration file
taskset -c $ARGOS_CORE \
argos3 -c ../experiments/convergence.argos & 
ARGOS_PID=$!

# ros2 topic hz /bot0/pose &
# HZ_PID=$!

# Wait for 5 minutes (300 seconds) before stopping the current iteration
sleep 6000

# Stop ARGoS3, ROS 2, rosbag, and RViz
echo "Stopping ARGoS3, ROS 2, rosbag, and RViz for iteration $i"
pkill -f argos3
pkill -f ros2
# pkill -f "ros2 bag"
pkill -f rviz2

# Wait for processes to terminate
wait $ROS2_PID
wait $ARGOS_PID

echo "All iterations completed"
