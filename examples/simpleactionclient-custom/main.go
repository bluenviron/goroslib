package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msg"
)

// define a custom action.
// unlike the standard library, a .action file is not needed.

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

	// send a goal
	err = sac.SendGoal(goroslib.SimpleActionClientGoalConf{
		Goal: &DoSomethingActionGoal{
			Input: 1234312,
		},
		OnDone: func(state goroslib.SimpleActionClientGoalState, res *DoSomethingActionResult) {
			fmt.Println("result:", res)
		},
		OnFeedback: func(fb *DoSomethingActionFeedback) {
			fmt.Println("feedback", fb)
		},
	})
	if err != nil {
		panic(err)
	}

	// freeze main loop
	select {}
}
