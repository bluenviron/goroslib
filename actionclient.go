package goroslib

import (
	"fmt"
	"reflect"
	"strconv"
	"sync"
	"time"

	"github.com/aler9/goroslib/pkg/action"
	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

// ActionClientGoalConf allows to configure SendGoal().
type ActionClientGoalConf struct {
	// the goal to send
	Goal interface{}

	// function called when a status transition happens
	OnTransition interface{}

	// function called when a feedback is received
	OnFeedback interface{}
}

// ActionClientGoalHandler is a goal handler.
type ActionClientGoalHandler struct {
	conf   ActionClientGoalConf
	status uint8
	result interface{}
}

// ActionClientConf is the configuration of an ActionClient.
type ActionClientConf struct {
	// node which the action client belongs to
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type
	Action interface{}
}

// ActionClient is a ROS action client, an entity that can call actions.
type ActionClient struct {
	conf           ActionClientConf
	goalActionType reflect.Type
	statusSub      *Subscriber
	feedbackSub    *Subscriber
	resultSub      *Subscriber
	goalPub        *Publisher
	cancelPub      *Publisher
	goalCount      int
	goals          map[string]*ActionClientGoalHandler
	mutex          sync.Mutex

	// in
	terminate chan struct{}

	// out
	statusSubOk   chan struct{}
	feedbackSubOk chan struct{}
	resultSubOk   chan struct{}
	goalPubOk     chan struct{}
	cancelPubOk   chan struct{}
}

// NewActionClient allocates an ActionClient. See ActionClientConf for the options.
func NewActionClient(conf ActionClientConf) (*ActionClient, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if conf.Name == "" {
		return nil, fmt.Errorf("Name is empty")
	}

	if conf.Action == nil {
		return nil, fmt.Errorf("Action is empty")
	}

	_, res, _, err := action.GoalResultFeedback(conf.Action)
	if err != nil {
		return nil, err
	}

	goalAction, resAction, fbAction, err := action.Messages(conf.Action)
	if err != nil {
		return nil, err
	}

	ac := &ActionClient{
		conf:          conf,
		goals:         make(map[string]*ActionClientGoalHandler),
		terminate:     make(chan struct{}),
		statusSubOk:   make(chan struct{}),
		feedbackSubOk: make(chan struct{}),
		resultSubOk:   make(chan struct{}),
		goalPubOk:     make(chan struct{}),
		cancelPubOk:   make(chan struct{}),
	}

	ac.goalActionType = reflect.TypeOf(goalAction)

	ac.statusSub, err = NewSubscriber(SubscriberConf{
		Node:  conf.Node,
		Topic: conf.Name + "/status",
		Callback: func(msg *actionlib_msgs.GoalStatusArray) {
			for _, status := range msg.StatusList {
				onTransition, dres := func() (interface{}, interface{}) {
					ac.mutex.Lock()
					defer ac.mutex.Unlock()

					data, ok := ac.goals[status.GoalId.Id]
					if !ok {
						return nil, nil
					}

					if status.Status != data.status {
						data.status = status.Status

						if data.conf.OnTransition != nil {
							dres := data.result

							if dres == nil {
								dres = reflect.New(reflect.TypeOf(res)).Interface()
							}

							return data.conf.OnTransition, dres
						}
					}

					return nil, nil
				}()

				if onTransition != nil {
					reflect.ValueOf(onTransition).Call([]reflect.Value{
						reflect.ValueOf(status.Status),
						reflect.ValueOf(dres),
					})
				}
			}

			select {
			case <-ac.statusSubOk:
				return
			default:
			}
			close(ac.statusSubOk)
		},
	})
	if err != nil {
		return nil, err
	}

	ac.feedbackSub, err = NewSubscriber(SubscriberConf{
		Node:  conf.Node,
		Topic: conf.Name + "/feedback",
		Callback: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{reflect.PtrTo(reflect.TypeOf(fbAction))}, []reflect.Type{}, false),
			func(in []reflect.Value) []reflect.Value {
				fbAction := in[0]

				goalStatus := fbAction.Elem().FieldByName("Status").
					Interface().(actionlib_msgs.GoalStatus)

				onFeedback := func() interface{} {
					ac.mutex.Lock()
					defer ac.mutex.Unlock()

					data, ok := ac.goals[goalStatus.GoalId.Id]
					if !ok {
						return nil
					}

					return data.conf.OnFeedback
				}()

				if onFeedback != nil {
					reflect.ValueOf(onFeedback).Call([]reflect.Value{
						fbAction.Elem().FieldByName("Feedback").Addr(),
					})
				}

				return []reflect.Value{}
			}).Interface(),
		onPublisher: func() {
			select {
			case <-ac.feedbackSubOk:
				return
			default:
			}
			close(ac.feedbackSubOk)
		},
	})
	if err != nil {
		ac.statusSub.Close()
		return nil, err
	}

	ac.resultSub, err = NewSubscriber(SubscriberConf{
		Node:  conf.Node,
		Topic: conf.Name + "/result",
		Callback: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{reflect.PtrTo(reflect.TypeOf(resAction))}, []reflect.Type{}, false),
			func(in []reflect.Value) []reflect.Value {
				resAction := in[0]

				goalStatus := resAction.Elem().FieldByName("Status").
					Interface().(actionlib_msgs.GoalStatus)

				func() {
					ac.mutex.Lock()
					defer ac.mutex.Unlock()

					data, ok := ac.goals[goalStatus.GoalId.Id]
					if !ok {
						return
					}

					data.result = resAction.Elem().FieldByName("Result").Addr().Interface()
				}()

				return []reflect.Value{}
			}).Interface(),
		onPublisher: func() {
			select {
			case <-ac.resultSubOk:
				return
			default:
			}
			close(ac.resultSubOk)
		},
	})
	if err != nil {
		ac.feedbackSub.Close()
		ac.statusSub.Close()
		return nil, err
	}

	ac.goalPub, err = NewPublisher(PublisherConf{
		Node:  conf.Node,
		Topic: conf.Name + "/goal",
		Msg:   reflect.New(reflect.TypeOf(goalAction)).Interface(),
		onSubscriber: func() {
			select {
			case <-ac.goalPubOk:
				return
			default:
			}
			close(ac.goalPubOk)
		},
	})
	if err != nil {
		ac.resultSub.Close()
		ac.feedbackSub.Close()
		ac.statusSub.Close()
		return nil, err
	}

	ac.cancelPub, err = NewPublisher(PublisherConf{
		Node:  conf.Node,
		Topic: conf.Name + "/cancel",
		Msg:   &actionlib_msgs.GoalID{},
		onSubscriber: func() {
			select {
			case <-ac.cancelPubOk:
				return
			default:
			}
			close(ac.cancelPubOk)
		},
	})
	if err != nil {
		ac.goalPub.Close()
		ac.resultSub.Close()
		ac.feedbackSub.Close()
		ac.statusSub.Close()
		return nil, err
	}

	return ac, nil
}

// Close closes an ActionClient and shuts down all its operations.
func (ac *ActionClient) Close() error {
	close(ac.terminate)
	ac.cancelPub.Close()
	ac.goalPub.Close()
	ac.resultSub.Close()
	ac.feedbackSub.Close()
	ac.statusSub.Close()
	return nil
}

// WaitForServer waits for the action server to start.
func (ac *ActionClient) WaitForServer() {
	select {
	case <-ac.terminate:
	case <-ac.statusSubOk:
	}

	select {
	case <-ac.terminate:
	case <-ac.feedbackSubOk:
	}

	select {
	case <-ac.terminate:
	case <-ac.resultSubOk:
	}

	select {
	case <-ac.terminate:
	case <-ac.goalPubOk:
	}

	select {
	case <-ac.terminate:
	case <-ac.cancelPubOk:
	}
}

// SendGoal sends a goal to the action server.
func (ac *ActionClient) SendGoal(conf ActionClientGoalConf) *ActionClientGoalHandler {
	action := reflect.New(ac.goalActionType)

	now := time.Now()

	header := std_msgs.Header{
		Stamp: now,
	}
	action.Elem().FieldByName("Header").Set(reflect.ValueOf(header))

	goalID := actionlib_msgs.GoalID{
		Stamp: now,
		Id: func() string {
			// https://github.com/ros/actionlib/blob/c3b2bd84f07ff54c36033c92861d3b63b7420590/actionlib/src/actionlib/goal_id_generator.py#L62
			ss := ac.conf.Node.absoluteName() + "-"
			ac.goalCount++
			ss += strconv.FormatInt(int64(ac.goalCount), 10) + "-"
			nowSecs := float64(now.UnixNano()) / 1000000000
			ss += strconv.FormatFloat(nowSecs, 'f', -1, 64)
			return ss
		}(),
	}
	action.Elem().FieldByName("GoalId").Set(reflect.ValueOf(goalID))

	action.Elem().FieldByName("Goal").Set(reflect.ValueOf(conf.Goal).Elem())

	ac.mutex.Lock()
	defer ac.mutex.Unlock()

	gh := &ActionClientGoalHandler{
		conf: conf,
	}
	ac.goals[goalID.Id] = gh

	ac.goalPub.Write(action.Interface())

	return gh
}
