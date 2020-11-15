// +build ignore

package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
)

func onMessage(msg *sensor_msgs.Imu) {
	fmt.Printf("Incoming: %+v\n", msg)
}

func main() {
	// create a node with given name and linked to given master.
	// master can be reached with an ip or hostname.
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/goroslib-sub",
		MasterHost: "127.0.0.1",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a subscriber, use the UDPROS protocol
	// the publisher must be based on roscpp or on another implementation that
	// supports the protocol. Currently, rospy (python) does not support it.
	sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "/test_pub",
		Callback: onMessage,
		Protocol: goroslib.UDP,
	})
	if err != nil {
		panic(err)
	}
	defer sub.Close()

	// freeze main loop
	select {}
}
