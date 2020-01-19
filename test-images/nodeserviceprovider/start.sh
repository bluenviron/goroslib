#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/

export ROS_IP=$(hostname -i)

source /catkin_ws/install/setup.bash

rosrun nodeserviceprovider nodeserviceprovider
