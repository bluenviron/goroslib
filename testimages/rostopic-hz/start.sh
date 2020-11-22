#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/
export ROS_IP=$(hostname -i)

stdbuf -oL -eL rostopic hz --window=5 /test_pub &

sleep 2
