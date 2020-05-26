#!/bin/bash -e

export ROS_MASTER_URI=http://$MASTER_IP:11311/
export ROS_IP=$(hostname -i)

rostopic pub -r 1 /test_pub sensor_msgs/Imu "header:
  seq: 13
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0]
angular_velocity: {x: 0.0, y: 0.0, z: 0.0}
angular_velocity_covariance: [0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: {x: 0.0, y: 0.0, z: 0.0}
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 13.7]"
