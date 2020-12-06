FROM ros:noetic-ros-core

RUN apt update && apt install -y --no-install-recommends \
    g++ \
    make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src/nodebusinfo2

COPY CMakeLists.txt main.cpp package.xml ./

WORKDIR /catkin_ws/src

COPY nodeservice_srv ./nodeservice_srv

WORKDIR /catkin_ws

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make install'

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
