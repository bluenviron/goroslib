FROM ros:melodic-ros-core

# to remove ASAP
# https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
RUN apt update; apt install -y --no-install-recommends \
    curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

COPY start.sh /
RUN chmod +x /start.sh

CMD [ "/start.sh" ]
