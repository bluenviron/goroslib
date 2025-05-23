// Package main contains an example.
package main

import (
	"os"
	"os/signal"
	"time"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

// define a custom action.
// unlike the standard library, an .action file is not needed.

type DoSomethingActionGoal struct { //nolint:revive
	Input uint32
}

type DoSomethingActionResult struct { //nolint:revive
	Output uint32
}

type DoSomethingActionFeedback struct { //nolint:revive
	PercentComplete float32
}

type DoSomethingAction struct { //nolint:revive
	msg.Package `ros:"shared_actions"`
	DoSomethingActionGoal
	DoSomethingActionResult
	DoSomethingActionFeedback
}

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a simple action server
	sas, err := goroslib.NewSimpleActionServer(goroslib.SimpleActionServerConf{
		Node:   n,
		Name:   "test_action",
		Action: &DoSomethingAction{},
		OnExecute: func(sas *goroslib.SimpleActionServer, _ *DoSomethingActionGoal) {
			// publish a feedback
			sas.PublishFeedback(&DoSomethingActionFeedback{
				PercentComplete: 0.5,
			})

			// wait some time
			time.Sleep(500 * time.Millisecond)

			// set the goal as succeeded
			sas.SetSucceeded(&DoSomethingActionResult{
				Output: 123456,
			})
		},
	})
	if err != nil {
		panic(err)
	}
	defer sas.Close()

	// wait for CTRL-C
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	<-c
}
