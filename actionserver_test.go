package goroslib

import (
	"reflect"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/actionproc"
	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
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

			switch client {
			case "cpp":
				c, err := newContainer("node-actionclient", m.IP())
				require.NoError(t, err)
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

				goalAction, resAction, fbAction, err := actionproc.Messages(&DoSomethingAction{})
				require.NoError(t, err)

				fbRecv := make(chan struct{})
				resRecv := make(chan struct{})
				statusRecv := make(chan struct{})

				statusSub, err := NewSubscriber(SubscriberConf{
					Node:  nc,
					Topic: "test_action/status",
					Callback: func(msg *actionlib_msgs.GoalStatusArray) {
						for _, s := range msg.StatusList {
							if s.GoalId.Id == "78910" && s.Status == 3 {
								close(statusRecv)
							}
						}
					},
				})
				require.NoError(t, err)
				defer statusSub.Close()

				feedbackSub, err := NewSubscriber(SubscriberConf{
					Node:  nc,
					Topic: "test_action/feedback",
					Callback: reflect.MakeFunc(
						reflect.FuncOf([]reflect.Type{reflect.PtrTo(reflect.TypeOf(fbAction))}, []reflect.Type{}, false),
						func(in []reflect.Value) []reflect.Value {
							require.Equal(t, DoSomethingActionFeedback{0.5}, in[0].Elem().FieldByName("Feedback").Interface())
							close(fbRecv)
							return nil
						},
					).Interface(),
				})
				require.NoError(t, err)
				defer feedbackSub.Close()

				resultSub, err := NewSubscriber(SubscriberConf{
					Node:  nc,
					Topic: "test_action/result",
					Callback: reflect.MakeFunc(
						reflect.FuncOf([]reflect.Type{reflect.PtrTo(reflect.TypeOf(resAction))}, []reflect.Type{}, false),
						func(in []reflect.Value) []reflect.Value {
							require.Equal(t, DoSomethingActionResult{123456}, in[0].Elem().FieldByName("Result").Interface())
							close(resRecv)
							return nil
						},
					).Interface(),
				})
				require.NoError(t, err)
				defer resultSub.Close()

				goalPub, err := NewPublisher(PublisherConf{
					Node:  nc,
					Topic: "test_action/goal",
					Msg:   reflect.New(reflect.TypeOf(goalAction)).Interface(),
				})
				require.NoError(t, err)
				defer goalPub.Close()

				time.Sleep(1 * time.Second)

				ga := reflect.New(reflect.TypeOf(goalAction))
				ga.Elem().FieldByName("Header").Set(reflect.ValueOf(std_msgs.Header{
					Stamp: time.Now(),
				}))
				ga.Elem().FieldByName("GoalId").Set(reflect.ValueOf(actionlib_msgs.GoalID{
					Stamp: time.Now(),
					Id:    "78910",
				}))
				ga.Elem().FieldByName("Goal").Set(reflect.ValueOf(&DoSomethingActionGoal{
					Input: 1234312,
				}).Elem())
				goalPub.Write(ga.Elem().Addr().Interface())

				<-fbRecv
				<-resRecv
				<-statusRecv
			}
		})
	}
}
