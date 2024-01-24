ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rosbridge-server && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
# "'topics_glob:=[*]'", "'services_glob:=[*]'", "'params_glob:=[*]'"