package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestActionServer(t *testing.T) {
	for _, client := range []string{
		"cpp",
		"go",
	} {
		t.Run(client, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

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

			switch client {
			case "cpp":
				c, err := newContainer("node-actionclient", m.IP())
				require.NoError(t, err)
				defer c.close()
				c.waitOutput()

			case "go":
				nc, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib-client",
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

				fb := <-feedDone1
				require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)

				res := <-resDone1
				require.Equal(t, &DoSomethingActionResult{123456}, res)
			}
		})
	}
}
