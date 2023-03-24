package main

import (
	"fmt"
	"os"
	"os/signal"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

// define a custom message.
// unlike the standard library, a .msg file is not needed.
// a structure definition is enough.
type TestMessage struct {
	msg.Package `ros:"my_package"`
	FirstField  uint32
	SecondField string
}

func main() {
	// create a node and connect to the master
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
		Msg:   &TestMessage{},
	})
	if err != nil {
		panic(err)
	}
	defer pub.Close()

	r := n.TimeRate(1 * time.Second)

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	for {
		select {
		// publish a message every second
		case <-r.SleepChan():
			msg := &TestMessage{
				FirstField:  3,
				SecondField: "test message",
			}
			fmt.Printf("Outgoing: %+v\n", msg)
			pub.Write(msg)

		// handle CTRL-C
		case <-c:
			return
		}
	}
}
