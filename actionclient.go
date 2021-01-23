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

// ActionClientCommState is the communication state of the goal of an action client.
type ActionClientCommState int

// standard goal communication states.
const (
	ActionClientCommStateWaitingForGoalAck ActionClientCommState = iota
	ActionClientCommStatePending
	ActionClientCommStateActive
	ActionClientCommStateWaitingForResult
	ActionClientCommStateWaitingForCancelAck
	ActionClientCommStateRecalling
	ActionClientCommStatePreempting
	ActionClientCommStateDone
	ActionClientCommStateLost
)

// String implements fmt.Stringer.
func (s ActionClientCommState) String() string {
	switch s {
	case ActionClientCommStateWaitingForGoalAck:
		return "waitingForGoalAck"
	case ActionClientCommStatePending:
		return "pending"
	case ActionClientCommStateActive:
		return "active"
	case ActionClientCommStateWaitingForResult:
		return "waitingForResult"
	case ActionClientCommStateWaitingForCancelAck:
		return "waitingForCancelAck"
	case ActionClientCommStateRecalling:
		return "recalling"
	case ActionClientCommStatePreempting:
		return "preempting"
	case ActionClientCommStateDone:
		return "done"
	default:
		return "lost"
	}
}

// ActionClientTerminalState is the terminal state of the goal of an action client.
type ActionClientTerminalState int

// standard goal terminal states.
const (
	ActionClientTerminalStateRecalled  ActionClientTerminalState = ActionClientTerminalState(actionlib_msgs.GoalStatus_RECALLED)
	ActionClientTerminalStateRejected  ActionClientTerminalState = ActionClientTerminalState(actionlib_msgs.GoalStatus_REJECTED)
	ActionClientTerminalStatePreempted ActionClientTerminalState = ActionClientTerminalState(actionlib_msgs.GoalStatus_PREEMPTED)
	ActionClientTerminalStateAborted   ActionClientTerminalState = ActionClientTerminalState(actionlib_msgs.GoalStatus_ABORTED)
	ActionClientTerminalStateSucceeded ActionClientTerminalState = ActionClientTerminalState(actionlib_msgs.GoalStatus_SUCCEEDED)
	ActionClientTerminalStateLost      ActionClientTerminalState = ActionClientTerminalState(actionlib_msgs.GoalStatus_LOST)
)

// String implements fmt.Stringer.
func (s ActionClientTerminalState) String() string {
	switch s {
	case ActionClientTerminalStateRecalled:
		return "recalled"
	case ActionClientTerminalStateRejected:
		return "rejected"
	case ActionClientTerminalStatePreempted:
		return "preempted"
	case ActionClientTerminalStateAborted:
		return "aborted"
	case ActionClientTerminalStateSucceeded:
		return "succeeded"
	default:
		return "lost"
	}
}

// ActionClientGoalConf allows to configure SendGoal().
type ActionClientGoalConf struct {
	// the goal to send
	Goal interface{}

	// function in the form func(*ActionClientGoalHandler, *ActionResult) that will be called when a status transition happens
	OnTransition interface{}

	// function in the form func(*ActionFeedback) that will becalled when a feedback is received
	OnFeedback interface{}
}

// ActionClientGoalHandler is a goal handler of an ActionClient.
type ActionClientGoalHandler struct {
	ac            *ActionClient
	conf          ActionClientGoalConf
	id            string
	commState     ActionClientCommState
	terminalState ActionClientTerminalState
	result        interface{}
}

// CommState returns the communication state of the goal handler.
func (gh *ActionClientGoalHandler) CommState() ActionClientCommState {
	return gh.commState
}

// TerminalState returns the terminal state of the goal handler.
func (gh *ActionClientGoalHandler) TerminalState() (ActionClientTerminalState, error) {
	if gh.commState != ActionClientCommStateDone {
		return 0, fmt.Errorf("unable to get terminal state when communication state is not Done")
	}

	return gh.terminalState, nil
}

func findStatus(statusList []actionlib_msgs.GoalStatus, id string) (uint8, bool) {
	for _, sta := range statusList {
		if sta.GoalId.Id == id {
			return sta.Status, true
		}
	}
	return 0, false
}

func (gh *ActionClientGoalHandler) onStatus(statusList []actionlib_msgs.GoalStatus) {
	if gh.commState == ActionClientCommStateDone {
		return
	}

	goalStatus, ok := findStatus(statusList, gh.id)
	if !ok {
		switch gh.commState {
		case ActionClientCommStateWaitingForGoalAck,
			ActionClientCommStateWaitingForResult:
		default:
			gh.terminalState = ActionClientTerminalStateLost
			gh.transitionTo(ActionClientCommStateDone)
		}
		return
	}

	switch gh.commState {
	case ActionClientCommStateWaitingForGoalAck:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
			gh.transitionTo(ActionClientCommStatePending)
		case actionlib_msgs.GoalStatus_ACTIVE:
			gh.transitionTo(ActionClientCommStateActive)
		case actionlib_msgs.GoalStatus_REJECTED:
			gh.transitionTo(ActionClientCommStatePending)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_RECALLING:
			gh.transitionTo(ActionClientCommStatePending)
			gh.transitionTo(ActionClientCommStateRecalling)
		case actionlib_msgs.GoalStatus_RECALLED:
			gh.transitionTo(ActionClientCommStatePending)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTED:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_SUCCEEDED:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_ABORTED:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTING:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStatePreempting)
		}

	case ActionClientCommStatePending:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
		case actionlib_msgs.GoalStatus_ACTIVE:
			gh.transitionTo(ActionClientCommStateActive)
		case actionlib_msgs.GoalStatus_REJECTED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_RECALLING:
			gh.transitionTo(ActionClientCommStateRecalling)
		case actionlib_msgs.GoalStatus_RECALLED:
			gh.transitionTo(ActionClientCommStateRecalling)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTED:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_SUCCEEDED:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_ABORTED:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTING:
			gh.transitionTo(ActionClientCommStateActive)
			gh.transitionTo(ActionClientCommStatePreempting)
		}

	case ActionClientCommStateActive:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
		case actionlib_msgs.GoalStatus_ACTIVE:
		case actionlib_msgs.GoalStatus_REJECTED:
		case actionlib_msgs.GoalStatus_RECALLING:
		case actionlib_msgs.GoalStatus_RECALLED:
		case actionlib_msgs.GoalStatus_PREEMPTED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_SUCCEEDED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_ABORTED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTING:
			gh.transitionTo(ActionClientCommStatePreempting)
		}

	case ActionClientCommStateWaitingForResult:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
		case actionlib_msgs.GoalStatus_ACTIVE:
		case actionlib_msgs.GoalStatus_REJECTED:
		case actionlib_msgs.GoalStatus_RECALLING:
		case actionlib_msgs.GoalStatus_RECALLED:
		case actionlib_msgs.GoalStatus_PREEMPTED:
		case actionlib_msgs.GoalStatus_SUCCEEDED:
		case actionlib_msgs.GoalStatus_ABORTED:
		case actionlib_msgs.GoalStatus_PREEMPTING:
		}

	case ActionClientCommStateWaitingForCancelAck:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
		case actionlib_msgs.GoalStatus_ACTIVE:
		case actionlib_msgs.GoalStatus_REJECTED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_RECALLING:
			gh.transitionTo(ActionClientCommStateRecalling)
		case actionlib_msgs.GoalStatus_RECALLED:
			gh.transitionTo(ActionClientCommStateRecalling)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_SUCCEEDED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_ABORTED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTING:
			gh.transitionTo(ActionClientCommStatePreempting)
		}

	case ActionClientCommStateRecalling:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
		case actionlib_msgs.GoalStatus_ACTIVE:
		case actionlib_msgs.GoalStatus_REJECTED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_RECALLING:
		case actionlib_msgs.GoalStatus_RECALLED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_SUCCEEDED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_ABORTED:
			gh.transitionTo(ActionClientCommStatePreempting)
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTING:
			gh.transitionTo(ActionClientCommStatePreempting)
		}

	case ActionClientCommStatePreempting:
		switch goalStatus {
		case actionlib_msgs.GoalStatus_PENDING:
		case actionlib_msgs.GoalStatus_ACTIVE:
		case actionlib_msgs.GoalStatus_REJECTED:
		case actionlib_msgs.GoalStatus_RECALLING:
		case actionlib_msgs.GoalStatus_RECALLED:
		case actionlib_msgs.GoalStatus_PREEMPTED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_SUCCEEDED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_ABORTED:
			gh.transitionTo(ActionClientCommStateWaitingForResult)
		case actionlib_msgs.GoalStatus_PREEMPTING:
		}
	}
}

func (gh *ActionClientGoalHandler) onFeedback(fbAction reflect.Value) {
	if gh.conf.OnFeedback != nil {
		reflect.ValueOf(gh.conf.OnFeedback).Call([]reflect.Value{
			fbAction.Elem().FieldByName("Feedback").Addr(),
		})
	}
}

func (gh *ActionClientGoalHandler) onResult(resAction reflect.Value) {
	switch gh.commState {
	case ActionClientCommStateWaitingForGoalAck,
		ActionClientCommStateWaitingForCancelAck,
		ActionClientCommStatePending,
		ActionClientCommStateActive,
		ActionClientCommStateWaitingForResult,
		ActionClientCommStateRecalling,
		ActionClientCommStatePreempting:

		gh.result = resAction.Elem().FieldByName("Result").Addr().Interface()
		gh.terminalState = ActionClientTerminalState(
			resAction.Elem().FieldByName("Status").FieldByName("Status").Interface().(uint8))
		gh.transitionTo(ActionClientCommStateDone)
	}
}

func (gh *ActionClientGoalHandler) transitionTo(newCommState ActionClientCommState) {
	gh.commState = newCommState

	if gh.conf.OnTransition != nil {
		dres := gh.result
		if dres == nil {
			dres = reflect.New(gh.ac.resType).Interface()
		}

		reflect.ValueOf(gh.conf.OnTransition).Call([]reflect.Value{
			reflect.ValueOf(gh),
			reflect.ValueOf(dres),
		})
	}
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
	goalType       reflect.Type
	resType        reflect.Type
	fbType         reflect.Type
	goalActionType reflect.Type
	resActionType  reflect.Type
	fbActionType   reflect.Type
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

	goal, res, fb, err := action.GoalResultFeedback(conf.Action)
	if err != nil {
		return nil, err
	}

	goalAction, resAction, fbAction, err := action.Messages(conf.Action)
	if err != nil {
		return nil, err
	}

	ac := &ActionClient{
		conf:           conf,
		goalType:       reflect.TypeOf(goal),
		resType:        reflect.TypeOf(res),
		fbType:         reflect.TypeOf(fb),
		goalActionType: reflect.TypeOf(goalAction),
		resActionType:  reflect.TypeOf(resAction),
		fbActionType:   reflect.TypeOf(fbAction),
		goals:          make(map[string]*ActionClientGoalHandler),
		terminate:      make(chan struct{}),
		statusSubOk:    make(chan struct{}),
		feedbackSubOk:  make(chan struct{}),
		resultSubOk:    make(chan struct{}),
		goalPubOk:      make(chan struct{}),
		cancelPubOk:    make(chan struct{}),
	}

	ac.statusSub, err = NewSubscriber(SubscriberConf{
		Node:     conf.Node,
		Topic:    conf.Name + "/status",
		Callback: ac.onStatus,
	})
	if err != nil {
		return nil, err
	}

	ac.feedbackSub, err = NewSubscriber(SubscriberConf{
		Node:  conf.Node,
		Topic: conf.Name + "/feedback",
		Callback: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{reflect.PtrTo(ac.fbActionType)}, []reflect.Type{}, false),
			ac.onFeedback,
		).Interface(),
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
			reflect.FuncOf([]reflect.Type{reflect.PtrTo(ac.resActionType)}, []reflect.Type{}, false),
			ac.onResult,
		).Interface(),
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
		Msg:   reflect.New(ac.goalActionType).Interface(),
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

func (ac *ActionClient) onStatus(msg *actionlib_msgs.GoalStatusArray) {
	func() {
		ac.mutex.Lock()
		defer ac.mutex.Unlock()

		for _, gh := range ac.goals {
			gh.onStatus(msg.StatusList)
		}
	}()

	select {
	case <-ac.statusSubOk:
		return
	default:
	}
	close(ac.statusSubOk)
}

func (ac *ActionClient) onFeedback(in []reflect.Value) []reflect.Value {
	fbAction := in[0]

	goalStatus := fbAction.Elem().FieldByName("Status").
		Interface().(actionlib_msgs.GoalStatus)

	ac.mutex.Lock()
	defer ac.mutex.Unlock()

	gh, ok := ac.goals[goalStatus.GoalId.Id]
	if !ok {
		return []reflect.Value{}
	}

	gh.onFeedback(fbAction)

	return []reflect.Value{}
}

func (ac *ActionClient) onResult(in []reflect.Value) []reflect.Value {
	resAction := in[0]

	goalStatus := resAction.Elem().FieldByName("Status").
		Interface().(actionlib_msgs.GoalStatus)

	ac.mutex.Lock()
	defer ac.mutex.Unlock()

	gh, ok := ac.goals[goalStatus.GoalId.Id]
	if !ok {
		return []reflect.Value{}
	}

	gh.onResult(resAction)

	return []reflect.Value{}
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
func (ac *ActionClient) SendGoal(conf ActionClientGoalConf) error {
	if conf.OnTransition != nil {
		cbt := reflect.TypeOf(conf.OnTransition)
		if cbt.Kind() != reflect.Func {
			return fmt.Errorf("OnTransition is not a function")
		}
		if cbt.NumIn() != 2 {
			return fmt.Errorf("OnTransition must accept a single argument")
		}
		if cbt.NumOut() != 0 {
			return fmt.Errorf("OnTransition must not return any value")
		}
		if cbt.In(0) != reflect.TypeOf(&ActionClientGoalHandler{}) {
			return fmt.Errorf("OnTransition 1st argument must be %s, while is %v",
				reflect.TypeOf(&ActionClientGoalHandler{}), cbt.In(0))
		}
		if cbt.In(1) != reflect.PtrTo(ac.resType) {
			return fmt.Errorf("OnTransition 2nd argument must be %s, while is %v",
				reflect.PtrTo(ac.resType), cbt.In(1))
		}
	}

	if conf.OnFeedback != nil {
		cbt := reflect.TypeOf(conf.OnFeedback)
		if cbt.Kind() != reflect.Func {
			return fmt.Errorf("OnFeedback is not a function")
		}
		if cbt.NumIn() != 1 {
			return fmt.Errorf("OnFeedback must accept a single argument")
		}
		if cbt.NumOut() != 0 {
			return fmt.Errorf("OnFeedback must not return any value")
		}
		if cbt.In(0) != reflect.PtrTo(ac.fbType) {
			return fmt.Errorf("OnFeedback 1st argument must must be %s, while is %v",
				reflect.PtrTo(ac.fbType), cbt.In(1))
		}
	}

	goalAction := reflect.New(ac.goalActionType)

	now := time.Now()

	header := std_msgs.Header{
		Stamp: now,
	}
	goalAction.Elem().FieldByName("Header").Set(reflect.ValueOf(header))

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
	goalAction.Elem().FieldByName("GoalId").Set(reflect.ValueOf(goalID))

	goalAction.Elem().FieldByName("Goal").Set(reflect.ValueOf(conf.Goal).Elem())

	ac.mutex.Lock()
	defer ac.mutex.Unlock()

	gh := &ActionClientGoalHandler{
		ac:   ac,
		conf: conf,
		id:   goalID.Id,
	}
	ac.goals[goalID.Id] = gh

	ac.goalPub.Write(goalAction.Interface())

	return nil
}
