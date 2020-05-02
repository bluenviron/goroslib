// +build ignore

package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
)

// define a custom message.
// unlike the standard library, a .msg file is not needed.
// a structure definition is enough.
type TestMessage struct {
	msgs.Package `ros:"my_package"`
	FirstField   msgs.Uint32
	SecondField  msgs.String
}

func onMessage(msg *TestMessage) {
	fmt.Printf("Incoming: %+v\n", msg)
}

func main() {
	// create a node with given name and linked to given master.
	// master can be reached with an ip or hostname.
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/goroslib",
		MasterHost: "127.0.0.1",
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
