package main

import (
	"fmt"
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

func onMessage(msg *std_msgs.Float64) {
	fmt.Printf("Incoming: %+v\n", msg)
}

func main() {
	time.Sleep(2 * time.Second)

	// create a node with given name and linked to given master.
	// master can be reached with an ip or hostname.
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/goroslib",
		MasterHost: "core1",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a subscriber
	sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "/test_pub",
		Callback: onMessage,
	})
	if err != nil {
		panic(err)
	}
	defer sub.Close()

	// freeze main loop
	infty := make(chan int)
	<-infty
}
