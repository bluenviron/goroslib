#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/
export ROS_IP=$(hostname -i)

C=0
stdbuf -oL -eL rostopic hz /test_pub | while read L; do
    if [ "$L" = "no new messages" ]; then
        continue
    fi

    echo "$L"

    C=$(($C + 1))
    if [ $C -ge 3 ]; then
        killall rostopic
    fi
done
