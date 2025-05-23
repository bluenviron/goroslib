// Package main contains an example.
package main

import (
	"log"
	"os"
	"os/signal"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

// TestMessage is a custom message.
// unlike the standard library, a .msg file is not needed.
type TestMessage struct {
	msg.Package `ros:"my_package"`
	FirstField  uint32
	SecondField string
}

func onMessage(msg *TestMessage) {
	log.Printf("Incoming: %+v\n", msg)
}

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_sub",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a subscriber
	sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "test_topic",
		Callback: onMessage,
	})
	if err != nil {
		panic(err)
	}
	defer sub.Close()

	// wait for CTRL-C
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	<-c
}
