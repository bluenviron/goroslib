// +build ignore

package main

import (
	"fmt"
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
)

func main() {
	// create a node and connects to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_pub",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a publisher
	pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "test_topic",
		Msg:   &sensor_msgs.Imu{},
	})
	if err != nil {
		panic(err)
	}
	defer pub.Close()

	// publish a message every second
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		msg := &sensor_msgs.Imu{
			AngularVelocity: geometry_msgs.Vector3{
				X: 23.5,
				Y: 22.1,
				Z: -7.5,
			},
		}
		fmt.Printf("Outgoing: %+v\n", msg)
		pub.Write(msg)
	}
}
