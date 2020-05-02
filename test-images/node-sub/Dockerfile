FROM ros:melodic-ros-core-bionic

WORKDIR /catkin_ws/src/nodesub

COPY CMakeLists.txt main.cpp package.xml ./
COPY msg ./msg

WORKDIR /catkin_ws

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make install'

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
