FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=$ARGOS_INSTALL/bin:$PATH
ENV LD_LIBRARY_PATH=$ARGOS_INSTALL/lib:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=$ARGOS_INSTALL/lib/pkgconfig
ENV ARGOS_PLUGIN_PATH=$ARGOS_INSTALL/lib/argos3
ENV ARGOS_CONFIG_PATH=$ARGOS_INSTALL/share/argos3
ENV ROS_WS=/ros2_ws

# Dependencies
RUN apt-get update && apt-get install -y \
     build-essential cmake git \
     python3-colcon-common-extensions python3-rosdep \
     libyaml-cpp-dev \
    ros-humble-rclcpp \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-core \
    ros-humble-std-msgs \
    ros-humble-yaml-cpp-vendor \
    ros-humble-angles \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-sensor-msgs \
    libfreeimage-dev libfreeimageplus-dev \
    qtbase5-dev libgsl-dev freeglut3-dev \
    libxi-dev libxmu-dev \
    liblua5.3-dev lua5.3 \
    doxygen graphviz libgraphviz-dev \
    asciidoc rsync gdb vim \
    && rm -rf /var/lib/apt/lists/*

# Layer 1: ARGoS3 (Single layer, suitable for caching)
RUN mkdir -p /tmp/argos_build && cd /tmp/argos_build \
    && git clone https://github.com/ilpincy/argos3.git \
    && cd argos3/src/plugins/robots \
    && rm -rf foot-bot \
    && git clone https://github.com/Tianfu-swarm/foot-bot.git \
    && cd /tmp/argos_build/argos3 \
    && mkdir build && cd build \
    && cmake ../src \
        -DARGOS_BUILD_FOR=simulator \
        -DARGOS_THREADSAFE_LOG=ON \
        -DARGOS_DYNAMIC_LOADING=ON \
    && make -j$(nproc) \
    && make doc \
    && make install \
    && rm -rf /tmp/argos_build

# Layer 2: argos3_ros_bridge
RUN mkdir -p $ROS_WS/src && cd $ROS_WS/src \
    && git clone https://github.com/Tianfu-swarm/argos3_ros_bridge.git \
    && cd $ROS_WS \
    && bash -c "source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install \
        --cmake-args \
            -DCMAKE_MODULE_PATH=/usr/local/share/argos3/cmake" \
    && rm -rf build log

RUN rosdep init || true && rosdep update

# Terminal Configuration
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc \
    && echo "alias cb='cd $ROS_WS && colcon build --symlink-install --packages-select fission_fusion_controller && source install/setup.bash'" >> /root/.bashrc \
    && echo "alias cba='cd \$ROS_WS && colcon build --symlink-install --cmake-args -DCMAKE_MODULE_PATH=/usr/local/share/argos3/cmake && source install/setup.bash'" >> /root/.bashrc \
    && echo "export PS1='[ROS2] \u:\w\$ '" >> /root/.bashrc

WORKDIR $ROS_WS
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

RUN ldconfig /usr/local/lib /usr/local/lib/argos3