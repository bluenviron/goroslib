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
				OnGoal: func(gh *ActionServerGoalHandler, goal *DoSomethingActionGoal) {
					go func() {
						if goal.Input == 1 {
							gh.SetRejected(&DoSomethingActionResult{})
							return
						}
						gh.SetAccepted()

						time.Sleep(500 * time.Millisecond)

						gh.PublishFeedback(&DoSomethingActionFeedback{
							PercentComplete: 0.5,
						})

						time.Sleep(500 * time.Millisecond)

						if goal.Input == 2 {
							gh.SetAborted(&DoSomethingActionResult{})
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
				require.Equal(t, "SUCCEEDED\n123456\n", c.waitOutput())

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

				feedDone := make(chan struct{})
				resDone := make(chan struct{})

				err = ac.SendGoal(ActionClientGoalConf{
					Goal: &DoSomethingActionGoal{
						Input: 1234312,
					},
					OnTransition: func(gh *ActionClientGoalHandler, res *DoSomethingActionResult) {
						if gh.CommState() == ActionClientCommStateDone {
							ts, err := gh.TerminalState()
							require.NoError(t, err)
							require.Equal(t, ActionClientTerminalStateSucceeded, ts)
							require.Equal(t, &DoSomethingActionResult{123456}, res)
							close(resDone)
						}
					},
					OnFeedback: func(fb *DoSomethingActionFeedback) {
						require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)
						close(feedDone)
					},
				})
				require.NoError(t, err)

				<-feedDone
				<-resDone
			}
		})
	}
}
