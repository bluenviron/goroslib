#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/
export ROS_IP=$(hostname -i)
export ROS_NAMESPACE=/myns

python3 -u /opt/ros/noetic/bin/rostopic hz --window=5 test_topic &

sleep 2
