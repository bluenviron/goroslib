package goroslib

import (
	"reflect"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/actionproc"
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
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

func writeFeedback(feedbackPub *Publisher, fbAction interface{}, goalID actionlib_msgs.GoalID,
	status uint8, fb interface{}) {
	fba := reflect.New(reflect.TypeOf(fbAction))
	fba.Elem().FieldByName("Header").Set(reflect.ValueOf(std_msgs.Header{
		Stamp: time.Now(),
	}))
	fba.Elem().FieldByName("Status").Set(reflect.ValueOf(actionlib_msgs.GoalStatus{
		GoalId: actionlib_msgs.GoalID{
			Id:    goalID.Id,
			Stamp: goalID.Stamp,
		},
		Status: status,
	}))
	fba.Elem().FieldByName("Feedback").Set(reflect.ValueOf(fb).Elem())
	feedbackPub.Write(fba.Interface())
}

func writeResult(resultPub *Publisher, resAction interface{}, goalID actionlib_msgs.GoalID,
	status uint8, res interface{}) {
	rea := reflect.New(reflect.TypeOf(resAction))
	rea.Elem().FieldByName("Header").Set(reflect.ValueOf(std_msgs.Header{
		Stamp: time.Now(),
	}))
	rea.Elem().FieldByName("Status").Set(reflect.ValueOf(actionlib_msgs.GoalStatus{
		GoalId: actionlib_msgs.GoalID{
			Id:    goalID.Id,
			Stamp: goalID.Stamp,
		},
		Status: status,
	}))
	rea.Elem().FieldByName("Result").Set(reflect.ValueOf(res).Elem())
	resultPub.Write(rea.Interface())
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

					goalAction, resAction, fbAction, err := actionproc.Messages(&DoSomethingAction{})
					require.NoError(t, err)

					statusPub, err := NewPublisher(PublisherConf{
						Node:  ns,
						Topic: "test_action/status",
						Msg:   &actionlib_msgs.GoalStatusArray{},
					})
					require.NoError(t, err)
					defer statusPub.Close()

					feedbackPub, err := NewPublisher(PublisherConf{
						Node:  ns,
						Topic: "test_action/feedback",
						Msg:   reflect.New(reflect.TypeOf(fbAction)).Interface(),
					})
					require.NoError(t, err)
					defer feedbackPub.Close()

					resultPub, err := NewPublisher(PublisherConf{
						Node:  ns,
						Topic: "test_action/result",
						Msg:   reflect.New(reflect.TypeOf(resAction)).Interface(),
					})
					require.NoError(t, err)
					defer resultPub.Close()

					goalSub, err := NewSubscriber(SubscriberConf{
						Node:  ns,
						Topic: "test_action/goal",
						Callback: reflect.MakeFunc(
							reflect.FuncOf([]reflect.Type{reflect.PtrTo(reflect.TypeOf(goalAction))}, []reflect.Type{}, false),
							func(in []reflect.Value) []reflect.Value {
								go func() {
									goalID := in[0].Elem().FieldByName("GoalId").
										Interface().(actionlib_msgs.GoalID)
									goal := in[0].Elem().FieldByName("Goal").Interface().(DoSomethingActionGoal)

									// reject
									if goal.Input == 1 {
										writeResult(resultPub, resAction, goalID,
											actionlib_msgs.GoalStatus_REJECTED, &DoSomethingActionResult{})
										return
									}

									// cancel
									if goal.Input == 3 {
										return
									}

									writeFeedback(feedbackPub, fbAction, goalID,
										actionlib_msgs.GoalStatus_ACTIVE, &DoSomethingActionFeedback{
											0.5,
										})

									// feedback must be received before result
									time.Sleep(500 * time.Millisecond)

									// abort
									if goal.Input == 2 {
										writeResult(resultPub, resAction, goalID,
											actionlib_msgs.GoalStatus_ABORTED, &DoSomethingActionResult{})
										return
									}

									writeResult(resultPub, resAction, goalID,
										actionlib_msgs.GoalStatus_SUCCEEDED, &DoSomethingActionResult{
											goal.Input + 1,
										})
								}()
								return nil
							},
						).Interface(),
					})
					require.NoError(t, err)
					defer goalSub.Close()

					cancelSub, err := NewSubscriber(SubscriberConf{
						Node:  ns,
						Topic: "test_action/cancel",
						Callback: func(msg *actionlib_msgs.GoalID) {
							writeResult(resultPub, resAction, *msg,
								actionlib_msgs.GoalStatus_PREEMPTED, &DoSomethingActionResult{})
						},
					})
					require.NoError(t, err)
					defer cancelSub.Close()
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
								require.Equal(t, &DoSomethingActionResult{1234312 + 1}, res)
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

func TestActionClientErrors(t *testing.T) {
	_, err := NewActionClient(ActionClientConf{})
	require.Error(t, err)

	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	_, err = NewActionClient(ActionClientConf{
		Node: n,
	})
	require.Error(t, err)

	_, err = NewActionClient(ActionClientConf{
		Node: n,
		Name: "myaction",
	})
	require.Error(t, err)

	_, err = NewActionClient(ActionClientConf{
		Node:   n,
		Name:   "myaction",
		Action: 123,
	})
	require.Error(t, err)
}
