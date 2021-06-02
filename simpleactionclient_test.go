package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestSimpleActionClient(t *testing.T) {
	for _, ca := range []string{
		"succeeded",
		"aborted",
		"canceled",
	} {
		for _, server := range []string{
			"cpp",
			"go",
		} {
			t.Run(ca+"_"+server, func(t *testing.T) {
				m, err := newContainerMaster()
				require.NoError(t, err)
				defer m.close()

				switch server {
				case "cpp":
					p, err := newContainer("node-simpleactionserver", m.IP())
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

					sas, err := NewSimpleActionServer(SimpleActionServerConf{
						Node:   ns,
						Name:   "test_action",
						Action: &DoSomethingAction{},
						OnExecute: func(sas *SimpleActionServer, goal *DoSomethingActionGoal) {
							if goal.Input == 3 {
								time.Sleep(2 * time.Second)
								if sas.IsPreemptRequested() {
									sas.SetAborted(&DoSomethingActionResult{})
									return
								}
							}

							time.Sleep(500 * time.Millisecond)

							sas.PublishFeedback(&DoSomethingActionFeedback{PercentComplete: 0.5})

							time.Sleep(500 * time.Millisecond)

							if goal.Input == 2 {
								sas.SetAborted(&DoSomethingActionResult{})
								return
							}

							sas.SetSucceeded(&DoSomethingActionResult{Output: 123456})
						},
					})
					require.NoError(t, err)
					defer sas.Close()
				}

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
					Goal: &DoSomethingActionGoal{
						Input: func() uint32 {
							switch ca {
							case "aborted":
								return 2
							case "canceled":
								return 3
							}
							return 1234312
						}(),
					},
					OnDone: func(state SimpleActionClientGoalState, res *DoSomethingActionResult) {
						switch ca {
						case "aborted":
							require.Equal(t, SimpleActionClientGoalStateAborted, state)

						case "canceled":
							require.Equal(t, SimpleActionClientGoalStateAborted, state)

						default:
							require.Equal(t, SimpleActionClientGoalStateSucceeded, state)
							require.Equal(t, &DoSomethingActionResult{123456}, res)
						}

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

				if ca == "canceled" {
					<-activeDone
					sac.CancelGoal()
					<-doneDone
				} else {
					<-activeDone
					<-fbDone
					<-doneDone
				}
			})
		}
	}
}

func TestSimpleActionClientGoalOverride(t *testing.T) {
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
				p, err := newContainer("node-simpleactionserver", m.IP())
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
			}

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

			err = sac.SendGoal(SimpleActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnDone: func(state SimpleActionClientGoalState, res *DoSomethingActionResult) {
					t.Errorf("should not happen")
				},
				OnActive: func() {
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
				},
			})
			require.NoError(t, err)

			done := make(chan struct{})

			err = sac.SendGoal(SimpleActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnDone: func(state SimpleActionClientGoalState, res *DoSomethingActionResult) {
					close(done)
				},
				OnActive: func() {
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
				},
			})
			require.NoError(t, err)

			<-done
		})
	}
}
