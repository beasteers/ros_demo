FROM osrf/ros:humble-desktop
RUN curl -o get-pip.py https://bootstrap.pypa.io/get-pip.py  && python3 get-pip.py
RUN python3 -m pip install -U fire tqdm Pillow opencv-python roslibpy foxglove-websocket attrs
ADD DDS_PROFILE.xml .
ENV FASTRTPS_DEFAULT_PROFILES_FILE DDS_PROFILE.xml
ADD *.py ./
