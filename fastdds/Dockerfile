ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

# RUN ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# ADD DDS_PROFILE.xml .
# ENV FASTRTPS_DEFAULT_PROFILES_FILE DDS_PROFILE.xml
EXPOSE 11811
CMD ["fastdds", "discovery", "--server-id", "0"]
# "fastdds discovery --server-id 0"