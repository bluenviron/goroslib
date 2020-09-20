FROM ros:melodic-ros-core-bionic

RUN apt update && apt install -y --no-install-recommends \
    g++ \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src/nodegen

COPY CMakeLists.txt main.cpp package.xml ./

WORKDIR /catkin_ws

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make install'

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
