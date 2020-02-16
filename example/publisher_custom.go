// +build ignore

package main

import (
	"fmt"
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
)

// define a custom message.
// unlike the standard library, a .msg file is not needed.
// a structure definition is enough.
type TestMessage struct {
	FirstField  msgs.Uint32
	SecondField msgs.String
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

	// create a publisher
	pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "/test_pub",
		Msg:   &TestMessage{},
	})
	if err != nil {
		panic(err)
	}
	defer pub.Close()

	// publish a message every second
	ticker := time.NewTicker(1 * time.Second)
	for range ticker.C {
		msg := &TestMessage{
			FirstField:  3,
			SecondField: "test message",
		}
		fmt.Printf("Outgoing: %+v\n", msg)
		pub.Write(msg)
	}
}
