#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/
export ROS_IP=$(hostname -i)
export ROS_NAMESPACE=/myns

rostopic echo -n 1 test_topic
