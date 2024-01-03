package main

import (
	"log"
	"os"
	"os/signal"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

// define a custom action.
// unlike the standard library, an .action file is not needed.

type DoSomethingActionGoal struct {
	Input uint32
}

type DoSomethingActionResult struct {
	Output uint32
}

type DoSomethingActionFeedback struct {
	PercentComplete float32
}

type DoSomethingAction struct {
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

	// create a simple action client
	sac, err := goroslib.NewSimpleActionClient(goroslib.SimpleActionClientConf{
		Node:   n,
		Name:   "test_action",
		Action: &DoSomethingAction{},
	})
	if err != nil {
		panic(err)
	}
	defer sac.Close()

	// wait for the server
	sac.WaitForServer()

	done := make(chan struct{})

	// send a goal
	err = sac.SendGoal(goroslib.SimpleActionClientGoalConf{
		Goal: &DoSomethingActionGoal{
			Input: 1234312,
		},
		OnDone: func(state goroslib.SimpleActionClientGoalState, res *DoSomethingActionResult) {
			log.Println("result:", res)
			close(done)
		},
		OnFeedback: func(fb *DoSomethingActionFeedback) {
			log.Println("feedback", fb)
		},
	})
	if err != nil {
		panic(err)
	}

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	select {
	// goal is done
	case <-done:

	// handle CTRL-C
	case <-c:
	}
}
