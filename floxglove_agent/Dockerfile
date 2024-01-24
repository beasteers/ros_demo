ARG ROS_DISTRO=humble
ARG AGENT_VERSION=1.0.3
ARG ARCH=arm64
FROM osrf/ros:${ROS_DISTRO}-desktop
# TODO: don't need ros here
# TODO: what command? look inside systemctl

RUN curl https://github.com/foxglove/agent/releases/download/v${AGENT_VERSION}/foxglove-agent_${AGENT_VERSION}_${ARCH}.deb foxglove-agent.deb && \
    dpkg -i foxglove-agent.deb

# CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml", "max_qos_depth:=1"]
