#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/
export ROS_IP=$(hostname -i)
export ROS_NAMESPACE=/myns

rosnode info goroslib 2>&1
