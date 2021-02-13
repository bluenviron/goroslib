package main

import (
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/actionlib"
)

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
		Action: &actionlib.TestAction{},
		OnExecute: func(sas *goroslib.SimpleActionServer, goal *actionlib.TestActionGoal) {
			// publish a feedback
			sas.PublishFeedback(&actionlib.TestActionFeedback{
				Feedback: 1,
			})

			// wait some time
			n.TimeSleep(500 * time.Millisecond)

			// set the goal as succeeded
			sas.SetSucceeded(&actionlib.TestActionResult{
				Result: 2,
			})
		},
	})
	if err != nil {
		panic(err)
	}
	defer sas.Close()

	// freeze main loop
	select {}
}
