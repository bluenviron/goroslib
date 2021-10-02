package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestSimpleActionServer(t *testing.T) {
	for _, client := range []string{
		"cpp",
		"go",
	} {
		t.Run(client, func(t *testing.T) {
			m := newContainerMaster(t)
			defer m.close()

			ns, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib-server",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer ns.Close()

			sas, err := NewSimpleActionServer(SimpleActionServerConf{
				Node:   ns,
				Name:   "test_action",
				Action: &DoSomethingAction{},
				OnExecute: func(sas *SimpleActionServer, goal *DoSomethingActionGoal) {
					time.Sleep(500 * time.Millisecond)

					sas.PublishFeedback(&DoSomethingActionFeedback{PercentComplete: 0.5})

					time.Sleep(500 * time.Millisecond)

					sas.SetSucceeded(&DoSomethingActionResult{Output: 123456})
				},
			})
			require.NoError(t, err)
			defer sas.Close()

			switch client {
			case "cpp":
				c := newContainer(t, "node-simpleactionclient", m.IP())
				defer c.close()
				require.Equal(t, "0.50\nSUCCEEDED\n123456\n", c.waitOutput())

			case "go":
				nc, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib-client",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer nc.Close()

				sac, err := NewSimpleActionClient(SimpleActionClientConf{
					Node:   nc,
					Name:   "test_action",
					Action: &DoSomethingAction{},
				})
				require.NoError(t, err)
				defer sac.Close()

				sac.WaitForServer()

				activeDone := make(chan struct{})
				fbDone := make(chan struct{})
				doneDone := make(chan struct{})

				err = sac.SendGoal(SimpleActionClientGoalConf{
					Goal: &DoSomethingActionGoal{Input: 1234312},
					OnDone: func(state SimpleActionClientGoalState, res *DoSomethingActionResult) {
						require.Equal(t, &DoSomethingActionResult{Output: 123456}, res)
						close(doneDone)
					},
					OnActive: func() {
						close(activeDone)
					},
					OnFeedback: func(fb *DoSomethingActionFeedback) {
						require.Equal(t, &DoSomethingActionFeedback{PercentComplete: 0.5}, fb)
						close(fbDone)
					},
				})
				require.NoError(t, err)

				<-activeDone
				<-fbDone
				<-doneDone
			}
		})
	}
}
