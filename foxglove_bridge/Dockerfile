ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-foxglove-bridge && rm -rf /var/lib/apt/lists/*
ADD DDS_PROFILE.xml .
ENV FASTRTPS_DEFAULT_PROFILES_FILE DDS_PROFILE.xml
# Run foxglove_bridge
CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml", "max_qos_depth:=1"]