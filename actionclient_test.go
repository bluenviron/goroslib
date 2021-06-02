package goroslib

import (
	"sync"
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
	for _, ca := range []string{
		"succeeded",
		"rejected",
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
						OnGoal: func(gh *ActionServerGoalHandler, goal *DoSomethingActionGoal) {
							go func() {
								if goal.Input == 1 {
									gh.SetRejected(&DoSomethingActionResult{})
									return
								}
								gh.SetAccepted()

								if goal.Input == 3 {
									return
								}

								time.Sleep(500 * time.Millisecond)

								gh.PublishFeedback(&DoSomethingActionFeedback{PercentComplete: 0.5})

								time.Sleep(500 * time.Millisecond)

								if goal.Input == 2 {
									gh.SetAborted(&DoSomethingActionResult{})
									return
								}

								gh.SetSucceeded(&DoSomethingActionResult{Output: 123456})
							}()
						},
						OnCancel: func(gh *ActionServerGoalHandler) {
							gh.SetCanceled(&DoSomethingActionResult{})
						},
					})
					require.NoError(t, err)
					defer as.Close()
				}

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

				gh, err := ac.SendGoal(ActionClientGoalConf{
					Goal: &DoSomethingActionGoal{
						Input: func() uint32 {
							switch ca {
							case "rejected":
								return 1
							case "aborted":
								return 2
							case "canceled":
								return 3
							}
							return 1234312
						}(),
					},
					OnTransition: func(gh *ActionClientGoalHandler, res *DoSomethingActionResult) {
						if gh.CommState() == ActionClientCommStateDone {
							ts, err := gh.TerminalState()
							require.NoError(t, err)

							switch ca {
							case "rejected":
								require.Equal(t, ActionClientTerminalStateRejected, ts)

							case "aborted":
								require.Equal(t, ActionClientTerminalStateAborted, ts)

							case "canceled":
								require.Equal(t, ActionClientTerminalStatePreempted, ts)

							default:
								require.Equal(t, ActionClientTerminalStateSucceeded, ts)
								require.Equal(t, &DoSomethingActionResult{123456}, res)
							}

							close(resDone)
						}
					},
					OnFeedback: func(fb *DoSomethingActionFeedback) {
						require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)
						close(feedDone)
					},
				})
				require.NoError(t, err)

				switch ca {
				case "succeeded",
					"aborted":
					<-feedDone

				case "canceled":
					time.Sleep(1 * time.Second)
					gh.Cancel()
				}

				<-resDone
			})
		}
	}
}

func TestActionClientDoubleGoal(t *testing.T) {
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
					OnGoal: func(gh *ActionServerGoalHandler, goal *DoSomethingActionGoal) {
						go func() {
							if goal.Input == 1 {
								gh.SetRejected(&DoSomethingActionResult{})
								return
							}
							gh.SetAccepted()

							if goal.Input == 3 {
								return
							}

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
						gh.SetCanceled(&DoSomethingActionResult{})
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

			var wg sync.WaitGroup
			wg.Add(4)

			_, err = ac.SendGoal(ActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnTransition: func(gh *ActionClientGoalHandler, res *DoSomethingActionResult) {
					if gh.CommState() == ActionClientCommStateDone {
						ts, err := gh.TerminalState()
						require.NoError(t, err)
						require.Equal(t, ActionClientTerminalStateSucceeded, ts)
						require.Equal(t, &DoSomethingActionResult{123456}, res)
						wg.Done()
					}
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
					require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)
					wg.Done()
				},
			})
			require.NoError(t, err)

			_, err = ac.SendGoal(ActionClientGoalConf{
				Goal: &DoSomethingActionGoal{
					Input: 1234312,
				},
				OnTransition: func(gh *ActionClientGoalHandler, res *DoSomethingActionResult) {
					if gh.CommState() == ActionClientCommStateDone {
						ts, err := gh.TerminalState()
						require.NoError(t, err)
						require.Equal(t, ActionClientTerminalStateSucceeded, ts)
						require.Equal(t, &DoSomethingActionResult{123456}, res)
						wg.Done()
					}
				},
				OnFeedback: func(fb *DoSomethingActionFeedback) {
					require.Equal(t, &DoSomethingActionFeedback{0.5}, fb)
					wg.Done()
				},
			})
			require.NoError(t, err)

			wg.Wait()
		})
	}
}
