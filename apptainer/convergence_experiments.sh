#!/bin/bash
# 容器内执行的脚本

# 检查是否传入了结果文件路径参数
if [ -z "$1" ]; then
    echo "Error: No results file path provided"
    exit 1
fi

RESULTS_FILE="$1"

# ==== 设置 ROS_DOMAIN_ID（从外部传入）====
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "[ERROR] ROS_DOMAIN_ID not provided from host. Aborting!"
    exit 1
else
    echo "[INFO] Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    export ROS_DOMAIN_ID
fi

source /opt/container_env/fission_fusion_controller_ws/install/setup.bash

# ROS 2 守护进程管理
ros2 daemon stop
ros2 daemon start

# 定义清理函数
cleanup() {
    echo "Stopping ARGoS3, ROS 2 and RViz..."
    pkill -f argos3
    pkill -f ros2
    pkill -f rviz2
    pkill -f "ros2 topic hz"
    exit 0
}

# 捕获中断信号
trap cleanup SIGINT SIGTERM

# 初始化环境
export LD_LIBRARY_PATH=/usr/local/lib/argos3:/opt/ros/humble/lib:/opt/container_env/fission_fusion_controller_ws/install/argos3_ros_bridge/lib:$LD_LIBRARY_PATH

export ARGOS_PLUGIN_PATH=/usr/local/lib/argos3:/opt/container_env/fission_fusion_controller_ws/install/argos3_ros_bridge/lib

# ==== CPU 分配 ====
ROS_CORES="0-41"
ARGOS_CORES="42-44"

# Disable Fast DDS shared memory transport to avoid /dev/shm permission issue
# export RMW_FASTRTPS_USE_SHM=OFF


# 启动 ROS 2 节点（绑定核心 0-41）
echo "[READY] ARGoS initialized, now launching ROS 2 Controller on cores $ROS_CORES"
taskset -c $ROS_CORES \
ros2 launch fission_fusion_controller run.launch.py numbers:=42.0 \
                                                    desired_subgroup_size:=14.0 \
                                                    follow_range:=10.0 \
                                                    subgroup_size_sigma:=0.0 \
                                                    groupsize_tolerance:=0.0 \
                                                    K:=800 \
                                                    early_converge_window:=8 \
                                                    isModelworks:=false \
                                                    isMinCommunication:=true \
                                                    isConCommunication:=true \
                                                    use_rviz:=true \
                                                    use_sim_time:=true \
                                                    results_file_path:=${results_path}&
ROS2_PID=$!
echo "[READY] Controller initialized"
sleep 1

# 启动 ARGoS3（绑定核心 42-44）
echo "Starting ARGoS3 on cores $ARGOS_CORES"
taskset -c $ARGOS_CORES \
argos3 -c /opt/container_env/fission_fusion_controller_ws/src/fission_fusion_controller/experiments/convergence.argos &
ARGOS_PID=$!

ros2 topic hz /bot0/pose &
HZ_PID=$!

echo "[READY] ARGoS initialized"

# 主运行周期 (10分钟)
echo "Running simulation for 600 seconds"
sleep 600

# 清理
echo "Stopping processes"
cleanup

echo "Simulation completed successfully"
exit 0