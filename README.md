
# goroslib

[![GoDoc](https://godoc.org/github.com/aler9/goroslib?status.svg)](https://godoc.org/github.com/aler9/goroslib)
[![Go Report Card](https://goreportcard.com/badge/github.com/aler9/goroslib)](https://goreportcard.com/report/github.com/aler9/goroslib)
[![Build Status](https://travis-ci.org/aler9/goroslib.svg?branch=master)](https://travis-ci.org/aler9/goroslib)

goroslib is a pure Go library for writing clients for the Robot Operating System (ROS). A ROS node connects to an existing master node and interacts with other nodes, exchanging structured data through topics, services and parameters.

The Robot Operating System (ROS) is a framework that defines a communication protocol between different programs, allowing them to exchange structured data, encoded in binary form. It was built for linking sensors, algorithms and actuators in unmanned ground vehicles and robots, but it is not bounded to the robot world and can be used anywhere there's need of building data streams, for instance in video processing.

The official project provides software for writing nodes in C++ and Python, but it requires the download of over 1GB of data and works only through a cmake-based buildchain, that is heavy and difficult to customize. This library allows to write lightweight nodes that can be built with the standard Go compiler, do not need any additional runtime library and have the size of some megabytes.

Features:
* Subscribe and publish to topics
* Call and provide services
* Get and set parameters
* Get infos about other nodes, topics, services
* Standard messages are precompiled and available in folder `msgs/`

The library provides its features by implementing in pure Go all the ROS protocols (xml-rpc, TCPROS) and APIs (Master API, Parameter Server API, Slave API).

## Installation

Go &ge; 1.12 is required, and modules must be enabled (i.e. there must be a file called `go.mod` in your project folder). To install the library, it is enough to write its name in the import section of the source files that will use it. Go will take care of downloading the needed files:
```go
import (
    "github.com/aler9/goroslib"
)
```

## Examples

* [subscriber](example/subscriber.go)
* [subscriber_custom_msg](example/subscriber_custom_msg.go)
* [publisher](example/publisher.go)
* [publisher_custom_msg](example/publisher_custom_msg.go)
* [serviceclient](example/serviceclient.go)
* [serviceprovider](example/serviceprovider.go)
* [param_get](example/param_get.go)
* [param_set](example/param_set.go)
* [cluster_info](example/cluster_info.go)

## Documentation

https://godoc.org/github.com/aler9/goroslib

## Links

Protocol documentation (v1)
* https://wiki.ros.orhttps://godoc.org/github.com/akio/rosgo/rosg/ROS/Technical%20Overview
* https://wiki.ros.org/Implementing%20Client%20Libraries
* https://wiki.ros.org/ROS/Master_API
* https://wiki.ros.org/ROS/Parameter%20Server%20API
* https://wiki.ros.org/ROS/Slave_API
* https://wiki.ros.org/ROS/Connection%20Header
* https://wiki.ros.org/ROS/TCPROS
* https://fossies.org/linux/wireshark/epan/dissectors/packet-tcpros.c

Messages
* https://github.com/ros/std_msgs
* https://github.com/ros/common_msgs

Other Go libraries
* (v1) https://github.com/akio/rosgo
* (v2) https://github.com/juaruipav/rclgo

Other non-Go libraries
* (v1) [cpp] https://github.com/ros/ros_comm/tree/melodic-devel/clients/roscpp/src/libros (https://docs.ros.org/melodic/api/roscpp/html/classros_1_1NodeHandle.html)
* (v1) [python] https://docs.ros.org/melodic/api/rosnode/html/
* (v1) [c] https://github.com/ros-industrial/cros
* (v2) [misc] https://fkromer.github.io/awesome-ros2/
