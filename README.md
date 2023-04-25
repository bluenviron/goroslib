# goroslib

[![Test](https://github.com/bluenviron/goroslib/workflows/test/badge.svg)](https://github.com/bluenviron/goroslib/actions?query=workflow:test)
[![Lint](https://github.com/bluenviron/goroslib/workflows/lint/badge.svg)](https://github.com/bluenviron/goroslib/actions?query=workflow:lint)
[![Go Report Card](https://goreportcard.com/badge/github.com/bluenviron/goroslib)](https://goreportcard.com/report/github.com/bluenviron/goroslib)
[![CodeCov](https://codecov.io/gh/bluenviron/goroslib/branch/main/graph/badge.svg)](https://app.codecov.io/gh/bluenviron/goroslib/branch/main)
[![PkgGoDev](https://pkg.go.dev/badge/github.com/bluenviron/goroslib/v2)](https://pkg.go.dev/github.com/bluenviron/goroslib/v2#pkg-index)

goroslib is a library in pure Go that allows to build clients (nodes) for the Robot Operating System (ROS).

The Robot Operating System (ROS) is a project that provides a specification to make multiple programs communicate with each other over time, exchanging structured data with topics, services, actions and parameters. It was conceived to link sensors, algorithms and actuators in unmanned ground vehicles (UGVs) and robots, but it is not bounded to the robot world and can be used anywhere there's the need of building streams of data (for example in video processing).

Features:

* publish and subscribe to topics with TCP or UDP
* provide and call services
* provide and call actions
* provide and call simple actions
* get and set parameters
* support namespaces and relative topics
* support IPv6 (stateful addresses only)
* support time API
* compilation of `.msg` files is not necessary, message definitions are extracted from code
* compile or cross-compile for all Go supported OSs (Linux, Windows, Mac OS X) and architectures
* examples provided for every feature, comprehensive test suite, continuous integration

## Table of contents

* [Installation](#installation)
* [API Documentation](#api-documentation)
* [FAQs](#faqs)
  * [Comparison with other libraries](#comparison-with-other-libraries)
  * [Full list of features](#full-list-of-features)
  * [Standard messages, services and actions](#standard-messages-services-and-actions)
  * [Custom messages, services and actions](#custom-messages-services-and-actions)
  * [Import existing messages, services and actions](#import-existing-messages-services-and-actions)
  * [Compile a node for another operating system](#compile-a-node-for-another-operating-system)
  * [Edit the library](#edit-the-library)
* [Standards](#standards)
* [Links](#links)

## Installation

1. Install Go &ge; 1.18.

2. Create an empty folder, open a terminal in it and initialize the Go modules system:

   ```
   go mod init main
   ```

3. Download one of the example files and place it in the folder:

   * [subscriber](examples/subscriber/main.go)
   * [subscriber-custom](examples/subscriber-custom/main.go)
   * [subscriber-udp](examples/subscriber-udp/main.go)
   * [subscriber-ipv6](examples/subscriber-ipv6/main.go)
   * [publisher](examples/publisher/main.go)
   * [publisher-custom](examples/publisher-custom/main.go)
   * [serviceclient](examples/serviceclient/main.go)
   * [serviceprovider](examples/serviceprovider/main.go)
   * [simpleactionclient](examples/simpleactionclient/main.go)
   * [simpleactionserver](examples/simpleactionserver/main.go)
   * [param-set-get](examples/param-set-get/main.go)
   * [cluster-info](examples/cluster-info/main.go)

4. Compile and run (a ROS master must be already running in the background)

   ```
   go run name-of-the-go-file.go
   ```

## API Documentation

https://pkg.go.dev/github.com/bluenviron/goroslib/v2#pkg-index

## FAQs

### Comparison with other libraries

#### goroslib vs official C++/Python libraries

The official project provides libraries to write nodes in C++ and Python, but they require the download of over 1GB of data and work only with a fixed buildchain. This library allows to write lightweight nodes that can be built with the standard Go compiler, do not need any runtime library and have a size of some megabytes. Another advantage lies in the possibility of compiling nodes for all the Golang supported operating systems (Linux, Windows, Mac OS X, etc) and architectures.

#### goroslib vs rosgo

rosgo is currently unmaintained; furthermore, it requires compilation of `.msg` files, doesn't support UDP, doesn't support actions, doesn't support simulated clocks.

### Full list of features

Current and missing features are [described in the FEATURES document](FEATURES.md).

### Standard messages, services and actions

This library provides most of the standard messages, services and actions in the folder `pkg/msgs`:

|package|documentation|repository|
|-------|-------------|----------|
|ackermann_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/ackermann_msgs)|[link](https://github.com/ros-drivers/ackermann_msgs)|
|actionlib|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/actionlib)|[link](https://github.com/ros/actionlib)|
|actionlib_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/actionlib_msgs)|[link](https://github.com/ros/common_msgs)|
|audio_common_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/audio_common_msgs)|[link](https://github.com/ros-drivers/audio_common)|
|control_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/control_msgs)|[link](https://github.com/ros-controls/control_msgs)|
|diagnostic_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/diagnostic_msgs)|[link](https://github.com/ros/common_msgs)|
|geometry_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs)|[link](https://github.com/ros/common_msgs)|
|geographic_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/geographic_msgs)|[link](https://github.com/ros-geographic-info/geographic_info)|
|mavros_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/mavros_msgs)|[link](https://github.com/mavlink/mavros)|
|nav_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/nav_msgs)|[link](https://github.com/ros/common_msgs)|
|rosgraph_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/rosgraph_msgs)|[link](https://github.com/ros/ros_comm_msgs)|
|sensor_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/sensor_msgs)|[link](https://github.com/ros/common_msgs)|
|shape_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/shape_msgs)|[link](https://github.com/ros/common_msgs)|
|sound_play|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/sound_play)|[link](https://github.com/ros-drivers/audio_common)|
|std_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs)|[link](https://github.com/ros/std_msgs)|
|std_srvs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/std_srvs)|[link](https://github.com/ros/ros_comm_msgs)|
|stereo_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/stereo_msgs)|[link](https://github.com/ros/common_msgs)|
|tf|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/tf)|[link](https://github.com/ros/geometry)|
|tf2_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/tf2_msgs)|[link](https://github.com/ros/geometry2)|
|trajectory_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/trajectory_msgs)|[link](https://github.com/ros/common_msgs)|
|uuid_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/uuid_msgs)|[link](https://github.com/ros-geographic-info/unique_identifier)|
|velodyne_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/velodyne_msgs)|[link](https://github.com/ros-drivers/velodyne)|
|vision_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/vision_msgs)|[link](https://github.com/ros-perception/vision_msgs)|
|visualization_msgs|[link](https://pkg.go.dev/github.com/bluenviron/goroslib/v2/pkg/msgs/visualization_msgs)|[link](https://github.com/ros/common_msgs)|

### Custom messages, services and actions

To define custom messages, the standard ROS C++/Python libraries require `.msg` files in this format:

```
bool field1
int32 field2
```

This library doesn't require any `.msg` file, it is enough to write Go structures in this format:

```go
import (
    "github.com/bluenviron/goroslib/v2/pkg/msgs"
)

type MessageName struct {
    msg.Package `ros:"my_package"`
    Field1 bool
    Field2 int32
}
```

The name of the message is taken from the name of the struct (in this case, `MessageName`), but it can be overridden by adding the `msg.Name` field:

```go
type MessageName struct {
    msg.Package `ros:"my_package"`
    msg.Name `ros:"my_message_name"`
    Field1 bool
    Field2 int32
}
```

The type of a field can be one of the following:

* one of the primitive field types: `bool`, `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`, `int64`, `uint64`, `float32`, `float64`, `string`, `time.Time`, `time.Duration`

* another standard or custom message

The name of a field must be in CamelCase, and is converted to snake_case when interacting with C++/Python nodes. If this conversion is not possible, the tag `rosname` can be used to override the field name:

```go
type MessageName struct {
    msg.Package `ros:"my_package"`
    Field bool  `rosname:"FIELD"`
}
```

Services in this format:

```
uint32 input
---
uint32 output
```

Are equivalent to Go structures in this format:

```go
type ServiceNameReq struct {
    Input uint32
}

type ServiceNameRes struct {
	Output uint32
}

type ServiceName struct {
	msg.Package `ros:"my_package"`
	ServiceNameReq
	ServiceNameRes
}
```

Actions in this format:

```
uint32 goal
---
uint32 result
---
uint32 feedback
```

Are equivalent to Go structures in this format:

```go
type ActionNameGoal struct {
	Goal uint32
}

type ActionNameResult struct {
	Result uint32
}

type ActionNameFeedback struct {
	Feedback uint32
}

type ActionName struct {
	msg.Package `ros:"my_package"`
	ActionNameGoal
	ActionNameResult
	ActionNameFeedback
}
```

### Import existing messages, services and actions

You can convert existing `.msg` files into their equivalent Go structures by using the `msg-import` tool:

```
go install github.com/bluenviron/goroslib/v2/cmd/msg-import@latest
msg-import --rospackage=my_package mymessage.msg > mymessage.go
```

You can convert existing `.srv` files into their equivalent Go structures by using the `srv-import` tool:

```
go install github.com/bluenviron/goroslib/v2/cmd/srv-import@latest
srv-import --rospackage=my_package myservice.srv > myservice.go
```

You can convert existing `.action` files into their equivalent Go structures by using the `action-import` tool:

```
go install github.com/bluenviron/goroslib/v2/cmd/action-import@latest
action-import --rospackage=my_package myaction.action > myaction.go
```

To convert a whole ROS package to Go, you can use the `package-import` tool:

```
go install github.com/bluenviron/goroslib/v2/cmd/package-import@latest
package-import <path-to-ros-package> --rospackage=my_package --gopackage=/pkg/msgs
```

### Compile a node for another operating system

To compile a node for another OS, it's enough to follow the standard Golang procedure to cross-compile, that consists in setting the `GOOS` and `GOARCH` environment variables according to the target machine. For instance, to build a node for Windows from another OS, run:

```
GOOS=windows GOARCH=amd64 go build -o node.exe name-of-source-file.go
```

### Edit the library

If you want to hack the library and test the results, unit tests can be launched with:

```
make test
```

## Standards

* ROS v1
  * General
    * [Technical overview](https://wiki.ros.org/ROS/Technical%20Overview)
    * [Implementing client libraries](https://wiki.ros.org/Implementing%20Client%20Libraries)
    * [Names](http://wiki.ros.org/Names)
    * [Actionlib](http://wiki.ros.org/actionlib)
    * [Actionlib detailed description](http://wiki.ros.org/actionlib/DetailedDescription)
  * APIs
    * [Master API](https://wiki.ros.org/ROS/Master_API)
    * [Parameter server API](https://wiki.ros.org/ROS/Parameter%20Server%20API)
    * [Slave API](https://wiki.ros.org/ROS/Slave_API)
  * Protocols
    * [Connection header](https://wiki.ros.org/ROS/Connection%20Header)
    * [TCPROS](https://wiki.ros.org/ROS/TCPROS)
    * [UDPROS](https://wiki.ros.org/ROS/UDPROS)
    * [Wireshark dissector](https://fossies.org/linux/wireshark/epan/dissectors/packet-prototcp.c)

* [Golang project layout](https://github.com/golang-standards/project-layout)

## Links

Other Go libraries

* [rosgo (v1)](https://github.com/akio/rosgo)
* [rclgo (v2)](https://github.com/juaruipav/rclgo)

Other non-Go libraries

* [official library (v1) (C++)](https://github.com/ros/ros_comm/tree/noetic-devel/clients/roscpp/src/libros) ([documentation](https://docs.ros.org/noetic/api/roscpp/html/classros_1_1NodeHandle.html))
* [official library (v1) (Python)](https://docs.ros.org/noetic/api/rosnode/html/)
* [cros (v1) (C)](https://github.com/ros-industrial/cros)
* [foxglove/ros1 (v1) (TypeScript)](https://github.com/foxglove/ros1)
* [awesome ros2 (v2) (misc)](https://fkromer.github.io/awesome-ros2/)
* [foxglove/ros2 (v2) (TypeScript)](https://github.com/foxglove/ros2)
