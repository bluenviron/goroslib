FROM ros:noetic-ros-core

# to remove ASAP
# https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
RUN apt update; apt install -y --no-install-recommends \
    curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt update && apt install -y --no-install-recommends \
    g++ \
    make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src/nodesub

COPY CMakeLists.txt main.cpp package.xml ./
COPY msg ./msg

WORKDIR /catkin_ws

RUN bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" \
    && catkin_make install'

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
