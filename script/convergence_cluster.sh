#!/bin/bash
APPTAINER=/home/scc/tianfu.zhang/apptainer_install/bin/apptainer
APPTAINER_CONF=/home/scc/tianfu.zhang/apptainer_install/etc/apptainer/apptainer.conf
SIF=/home/scc/tianfu.zhang/fission_fusion.sif
WS=/home/scc/tianfu.zhang/ros2_ws

ALPHA=${1:-8.0}
BETA=${2:-1.0}
RESULTS_PATH=${3:-"$WS/src/fission_fusion_controller/data/result_$(date +"%Y%m%d%H%M")"}

mkdir -p "$RESULTS_PATH"
echo "ALPHA=$ALPHA BETA=$BETA RESULTS_PATH=$RESULTS_PATH"

$APPTAINER -c $APPTAINER_CONF exec --cleanenv --bind $WS:/ros2_ws $SIF bash -c '
    echo "PYTHONPATH before: $PYTHONPATH"
    unset PYTHONPATH
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    echo "PYTHONPATH after: $PYTHONPATH"
    python3 -c "import ros2cli; print(ros2cli.__file__)"

    export ARGOS_PLUGIN_PATH=/ros2_ws/install/argos3_ros_bridge/lib
    export LD_PRELOAD=$(find /usr/local/lib/argos3 -name "*.so" | tr "\n" ":")
    export RMW_FASTRTPS_USE_SHM=OFF
    export ROS_DOMAIN_ID=$((($(date +%s) + $$) % 233))

    EXPERIMENTS_DIR=/ros2_ws/src/fission_fusion_controller/experiments
    TMP_ARGOS=/tmp/argos_$$.argos
    sed "s|EXPERIMENTS_DIR|$EXPERIMENTS_DIR|g" \
        $EXPERIMENTS_DIR/convergence.argos > $TMP_ARGOS
    echo "TMP_ARGOS=$TMP_ARGOS"

    argos3 -c $TMP_ARGOS &
    ARGOS_PID=$!
    echo "ARGOS_PID=$ARGOS_PID"

    sleep 2
    /opt/ros/humble/bin/ros2 launch fission_fusion_controller run.launch.py \
        numbers:=42.0 \
        desired_subgroup_size:=14.0 \
        follow_range:=5.0 \
        subgroup_size_sigma:=0.0 \
        groupsize_tolerance:=0.0 \
        K:=1000 \
        early_converge_window:=8 \
        isModelworks:=false \
        isMinCommunication:=true \
        isConCommunication:=true \
        use_rviz:=false \
        use_sim_time:=true \
        alpha:='"$ALPHA"' \
	T_max:=100.0 \
        beta:='"$BETA"' \
        results_file_path:='"$RESULTS_PATH"' &
    ROS2_PID=$!

    sleep 900

    kill $ROS2_PID $ARGOS_PID 2>/dev/null
    wait $ROS2_PID $ARGOS_PID 2>/dev/null
    echo "Done"
'
