package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msg"
)

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

func TestActionClient(t *testing.T) {
	for _, server := range []string{
		"cpp",
		"go",
	} {
		t.Run(server, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch server {
			case "cpp":
				p, err := newContainer("node-actionserver", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				ns, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib-server",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer ns.Close()

				as, err := NewActionServer(ActionServerConf{
					Node:   ns,
					Name:   "test_action",
					Action: &DoSomethingAction{},
					OnGoal: func(goal *DoSomethingActionGoal, gh *ActionServerGoalHandler) {
						go func() {
							if goal.Input == 1 {
								gh.SetRejected()
								return
							}
							gh.SetAccepted()

							time.Sleep(1 * time.Second)

							gh.PublishFeedback(&DoSomethingActionFeedback{
								PercentComplete: 0.5,
							})

							time.Sleep(1 * time.Second)

							if goal.Input == 2 {
								gh.SetAborted()
								return
							}

							gh.SetSucceeded(&DoSomethingActionResult{
								Output: 123456,
							})
						}()
					},
					OnCancel: func(gh *ActionServerGoalHandler) {
					},
				})
				require.NoError(t, err)
				defer as.Close()
			}

			nc, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer nc.Close()

			ac, err := NewActionClient(ActionClientConf{
				Node:   nc,
				Name:   "test_action",
				Action: &DoSomethingAction{},
			})
			require.NoError(t, err)
			defer ac.Close()

			ac.WaitForServer()

			feedDone1 := make(chan *DoSomethingActionFeedback, 1)
			resDone1 := make(chan *DoSomethingActionResult, 1)

			ac.SendGoal(ActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnTransition: func(status ActionGoalStatus, res *DoSomethingActionResult) {
					if status == Succeeded {
						resDone1 <- res
					}
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
					feedDone1 <- fb
				},
			})

			feedDone2 := make(chan *DoSomethingActionFeedback, 1)
			resDone2 := make(chan *DoSomethingActionResult, 1)

			err = ac.SendGoal(ActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnTransition: func(status ActionGoalStatus, res *DoSomethingActionResult) {
					if status == Succeeded {
						resDone2 <- res
					}
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
					feedDone2 <- fb
				},
			})
			require.NoError(t, err)

			fb := <-feedDone1
			require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)

			res := <-resDone1
			require.Equal(t, &DoSomethingActionResult{123456}, res)

			fb = <-feedDone2
			require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)

			res = <-resDone2
			require.Equal(t, &DoSomethingActionResult{123456}, res)
		})
	}
}
