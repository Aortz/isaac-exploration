# OpenVINS (ROS 2 Jazzy) for the Isaac fisheye rig. Build: ros2/openvins/build.sh
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
        git libeigen3-dev libboost-all-dev libceres-dev \
        ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
ARG OPENVINS_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b
WORKDIR /ws/src
RUN git clone https://github.com/rpng/open_vins.git && git -C open_vins checkout ${OPENVINS_COMMIT}
# Jazzy dropped the deprecated .h headers that OpenVINS (written for Humble) still includes.
RUN grep -rlE "(image_transport/image_transport|tf2_geometry_msgs/tf2_geometry_msgs|cv_bridge/cv_bridge)\.h>" open_vins \
    | xargs sed -i -E 's#(image_transport/image_transport|tf2_geometry_msgs/tf2_geometry_msgs|cv_bridge/cv_bridge)\.h>#\1.hpp>#'
# The IMU subscription is best-effort with depth 5; the simulator publishes IMU in bursts between
# frames, so samples get dropped. Make it reliable with a deep queue.
RUN sed -i 's#create_subscription<sensor_msgs::msg::Imu>(topic_imu, rclcpp::SensorDataQoS(),#create_subscription<sensor_msgs::msg::Imu>(topic_imu, rclcpp::QoS(rclcpp::KeepLast(2000)).reliable(),#' \
        open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp \
    && grep -q "KeepLast(2000)).reliable()" open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp
# After init, OpenVINS clones the state at queued camera times. With 4 separate mono cameras sharing a
# stamp the queue holds each time 4x; cloning a time equal to the state's calls exit(). Skip just those
# (queue and clone rate stay as upstream). Four-camera runs are still not repeatable run to run; see
# docs/experiments/openvins_multicam.md.
RUN sed -i 's#        propagator->propagate_and_clone(state, camera_timestamps_to_init.at(i));#        if (camera_timestamps_to_init.at(i) > state->_timestamp) propagator->propagate_and_clone(state, camera_timestamps_to_init.at(i));#' \
        open_vins/ov_msckf/src/core/VioManagerHelper.cpp \
    && grep -q "camera_timestamps_to_init.at(i) > state->_timestamp" open_vins/ov_msckf/src/core/VioManagerHelper.cpp
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select ov_core ov_init ov_msckf \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4 && rm -rf build log
RUN echo ". /opt/ros/jazzy/setup.bash && . /ws/install/setup.bash" >> /etc/bash.bashrc
