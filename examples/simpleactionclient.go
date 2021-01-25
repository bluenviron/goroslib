// +build ignore

package main

import (
	"fmt"

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

	// create a simple action client
	sac, err := goroslib.NewSimpleActionClient(goroslib.SimpleActionClientConf{
		Node:   n,
		Name:   "test_action",
		Action: &actionlib.TestAction{},
	})
	if err != nil {
		panic(err)
	}
	defer sac.Close()

	// wait for the server
	sac.WaitForServer()

	// send a goal
	err = sac.SendGoal(goroslib.SimpleActionClientGoalConf{
		Goal: &actionlib.TestActionGoal{
			Goal: 1234312,
		},
		OnDone: func(res *actionlib.TestActionResult) {
			fmt.Println("result:", res)
		},
		OnFeedback: func(fb *actionlib.TestActionFeedback) {
			fmt.Println("feedback", fb)
		},
	})
	if err != nil {
		panic(err)
	}

	// freeze main loop
	select {}
}
