
# goroslib

[![Test](https://github.com/aler9/goroslib/workflows/test/badge.svg)](https://github.com/aler9/goroslib/actions)
[![Lint](https://github.com/aler9/goroslib/workflows/lint/badge.svg)](https://github.com/aler9/goroslib/actions)
[![PkgGoDev](https://pkg.go.dev/badge/github.com/aler9/goroslib)](https://pkg.go.dev/github.com/aler9/goroslib)

goroslib is a library in pure Go that allows to build clients (nodes) for the Robot Operating System (ROS).

The Robot Operating System (ROS) is a project that provides a protocol specification to make multiple programs communicate with each other over time, exchanging structured data through topics, services and parameters. It was conceived to link sensors, algorithms and actuators in unmanned ground vehicles (UGVs) and robots, but it is not bounded to the robot world and can be used anywhere there's the need of building streams of data (for example in video processing).

The official project provides libraries to write nodes in C++ and Python, but they require the download of over 1GB of data and work only through a cmake-based buildchain, that is computationally intensive and difficult to customize. This library allows to write lightweight nodes that can be built with the standard Go compiler, do not need any runtime library and have a size of some megabytes. Another advantage lies in the possibility of compiling nodes for all the Golang supported operating systems (Linux, Windows, Mac OS X, etc) and architectures.

Features:

* Subscribe and publish to topics, with TCP or UDP
* Call and provide services
* Get and set parameters
* Get infos about other nodes, topics, services
* Use namespaces and relative topics
* IPv6 support (only stateful addresses, since stateless are not supported by the ROS master)
* Compilation of `.msg` files is not necessary, message definitions are extracted from code
* Standard messages are available in folder `msgs/`
* Compile or cross-compile ROS nodes for all Golang supported OSs (Linux, Windows, Mac OS X) and architectures
* Examples provided for every feature, comprehensive test suite, continuous integration

The library provides its features by implementing in pure Go all the ROS protocols (xml-rpc, TCPROS, UDPROS) and APIs (Master API, Parameter Server API, Slave API).

All the official [client libraries requirements](https://wiki.ros.org/Implementing%20Client%20Libraries) are satisfied, except for the following ones:

* parse command-line Remapping Arguments
* Subscribe to a simulated Clock
* publish debugging messages to rosout

## Installation

1. Install Go &ge; 1.13.

2. Create an empty folder, open a terminal in it and initialize the Go modules system:

   ```
   go mod init main
   ```

3. Download one of the example files and place it in the folder:

   * [subscriber](examples/subscriber.go)
   * [subscriber-custom](examples/subscriber-custom.go)
   * [subscriber-udp](examples/subscriber-udp.go)
   * [subscriber-ipv6](examples/subscriber-ipv6.go)
   * [publisher](examples/publisher.go)
   * [publisher-custom](examples/publisher-custom.go)
   * [serviceclient](examples/serviceclient.go)
   * [serviceprovider](examples/serviceprovider.go)
   * [param-set-get](examples/param-set-get.go)
   * [cluster-info](examples/cluster-info.go)

4. Compile and run (a ROS master must be already running in the background)

   ```
   go run name-of-the-go-file.go
   ```

## Documentation

https://pkg.go.dev/github.com/aler9/goroslib

## FAQs

### Where can i find the standard messages?

Standard messages are [listed in the documentation](https://pkg.go.dev/github.com/aler9/goroslib/pkg/msgs?tab=subdirectories).

### How can i define a custom message?

To define custom messages, the standard ROS C++/Python libraries require `.msg` files in this format:

```
bool field1
int32 field2
```

This library doesn't require any `.msg` file, it is enough to write Go structures in this format:

```go
import (
    "github.com/aler9/goroslib/pkg/msgs"
)

type MessageName struct {
    msg.Package `ros:"my_package"`
    Field1 bool
    Field2 int32
}
```

The type of a field can be one of the following:

* one of the primitive field types:

  * bool
  * int8
  * uint8
  * int16
  * uint16
  * int32
  * uint32
  * int64
  * uint64
  * float32
  * float64
  * string
  * time.Time
  * time.Duration

* another standard or custom message

The name of a field must be in CamelCase, and is converted to snake_case when interacting with C++/Python nodes. If this conversion is impossible, the tag `rosname` can be used to override the field name:

```go
type MessageName struct {
    msg.Package `ros:"my_package"`
    Field bool  `rosname:"FIELD"`
}
```

A command-line utility is provided to convert existing `.msg` files into their equivalent Go structures:

```
go get github.com/aler9/goroslib/cmd/msg-import
msg-import --rospackage=my_package mymessage.msg > mymessage.go
```

### How can i set the namespace?

There's a field `Namespace` in the `Node` configuration:

```go
goroslib.NewNode(goroslib.NodeConf{
    Namespace:     "/mynamespace",
    Name:          "goroslib",
    MasterAddress: "127.0.0.1:11311",
})
```

The default namespace is `/` (global namespace).

### How can i compile a node for another operating system?

To compile a node for another OS, it's enough to follow the standard Golang procedure to cross-compile, that consists in setting the `GOOS` and `GOARCH` environment variables according to the target machine. For instance, to build a node for Windows from another OS, run:

```
GOOS=windows GOARCH=amd64 go build -o node.exe name-of-source-file.go
```

### How can i edit the library?

If you want to hack the library and test the results, unit tests can be launched with:

```
make test
```

## Links

(v1) ROS Documentation

* Conventions
  * https://wiki.ros.org/ROS/Technical%20Overview
  * https://wiki.ros.org/Implementing%20Client%20Libraries
  * http://wiki.ros.org/Names
* APIs
  * https://wiki.ros.org/ROS/Master_API
  * https://wiki.ros.org/ROS/Parameter%20Server%20API
  * https://wiki.ros.org/ROS/Slave_API
* Protocols
  * https://wiki.ros.org/ROS/Connection%20Header
  * https://wiki.ros.org/ROS/TCPROS
  * https://wiki.ros.org/ROS/UDPROS
  * https://fossies.org/linux/wireshark/epan/dissectors/packet-prototcp.c
* Messages
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

Conventions

* https://github.com/golang-standards/project-layout
