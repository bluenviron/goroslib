FROM ros:noetic-ros-core

RUN apt update && apt install -y --no-install-recommends \
    g++ \
    make \
    ros-noetic-actionlib \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src/nodesimpleactionserver

COPY CMakeLists.txt main.cpp package.xml ./

WORKDIR /catkin_ws/src

COPY shared_actions ./shared_actions

WORKDIR /catkin_ws

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make install'

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
