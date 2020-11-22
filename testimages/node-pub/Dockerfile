FROM ros:noetic-ros-core

RUN apt update && apt install -y --no-install-recommends \
    g++ \
    make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src/nodepub

COPY CMakeLists.txt main.cpp package.xml ./
COPY msg ./msg

WORKDIR /catkin_ws

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make install'

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
