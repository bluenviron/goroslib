package goroslib

import (
	"fmt"
	"reflect"
	"sync"
)

type simpleActionClientGoalSimpleState int

const (
	simpleActionClientGoalSimpleStatePending simpleActionClientGoalSimpleState = iota
	simpleActionClientGoalSimpleStateActive
	simpleActionClientGoalSimpleStateDone
)

// SimpleActionClientGoalState is the state of a goal of a simple action client.
type SimpleActionClientGoalState int

// standard goal states.
const (
	SimpleActionClientGoalStatePending SimpleActionClientGoalState = iota
	SimpleActionClientGoalStateActive
	SimpleActionClientGoalStateRecalled
	SimpleActionClientGoalStateRejected
	SimpleActionClientGoalStatePreempted
	SimpleActionClientGoalStateAborted
	SimpleActionClientGoalStateSucceeded
	SimpleActionClientGoalStateLost
)

type simpleActionClientGoalHandler struct {
	conf  SimpleActionClientGoalConf
	state simpleActionClientGoalSimpleState
	gh    *ActionClientGoalHandler
}

// SimpleActionClientConf is the configuration of a SimpleActionClient.
type SimpleActionClientConf struct {
	// parent node.
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type.
	Action interface{}
}

// SimpleActionClient is a ROS simple action client, an entity that can call simple actions.
type SimpleActionClient struct {
	ac    *ActionClient
	mutex sync.Mutex
	sgh   *simpleActionClientGoalHandler
}

// NewSimpleActionClient allocates a SimpleActionClient.
func NewSimpleActionClient(conf SimpleActionClientConf) (*SimpleActionClient, error) {
	ac, err := NewActionClient(ActionClientConf(conf))
	if err != nil {
		return nil, err
	}

	sac := &SimpleActionClient{
		ac: ac,
	}

	return sac, nil
}

// Close closes a SimpleActionClient.
func (sac *SimpleActionClient) Close() error {
	sac.ac.Close()
	return nil
}

// WaitForServer waits for the action server to start.
func (sac *SimpleActionClient) WaitForServer() {
	sac.ac.WaitForServer()
}

// SimpleActionClientGoalConf is the configuration of SendGoal().
type SimpleActionClientGoalConf struct {
	// the goal to send.
	Goal interface{}

	// (optional) function in the form func(*ActionResult) that will be called
	// when the goal is done.
	OnDone interface{}

	// (optional) function that will be called when the goal becomes active.
	OnActive func()

	// (optional) function in the form func(*ActionFeedbacK) that will be
	// called when a feedback arrives.
	OnFeedback interface{}
}

// SendGoal sends a goal.
func (sac *SimpleActionClient) SendGoal(conf SimpleActionClientGoalConf) error {
	if conf.OnDone != nil {
		cbt := reflect.TypeOf(conf.OnDone)
		if cbt.Kind() != reflect.Func {
			return fmt.Errorf("OnDone is not a function")
		}

		if cbt.NumIn() != 2 {
			return fmt.Errorf("OnDone must accept 2 arguments")
		}
		if cbt.In(0) != reflect.TypeOf(SimpleActionClientGoalState(0)) {
			return fmt.Errorf("OnDone 1st argument must be %s, while is %v",
				reflect.TypeOf(SimpleActionClientGoalState(0)), cbt.In(0))
		}
		if cbt.In(1) != reflect.PtrTo(sac.ac.resType) {
			return fmt.Errorf("OnDone 2nd argument must be %s, while is %v",
				reflect.PtrTo(sac.ac.resType), cbt.In(1))
		}

		if cbt.NumOut() != 0 {
			return fmt.Errorf("OnDone must not return any value")
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
		if cbt.In(0) != reflect.PtrTo(sac.ac.fbType) {
			return fmt.Errorf("OnFeedback 1st argument must be %s, while is %v",
				reflect.PtrTo(sac.ac.fbType), cbt.In(0))
		}

		if cbt.NumOut() != 0 {
			return fmt.Errorf("OnFeedback must not return any value")
		}
	}

	sac.mutex.Lock()
	defer sac.mutex.Unlock()

	sac.sgh = &simpleActionClientGoalHandler{
		conf: conf,
	}

	fixedSGH := sac.sgh

	gh, err := sac.ac.SendGoal(ActionClientGoalConf{
		Goal: conf.Goal,
		OnTransition: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{
				reflect.TypeOf(&ActionClientGoalHandler{}),
				reflect.PtrTo(sac.ac.resType),
			}, []reflect.Type{}, false),
			func(in []reflect.Value) []reflect.Value {
				return sac.onTransition(fixedSGH, in)
			},
		).Interface(),
		OnFeedback: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{
				reflect.PtrTo(sac.ac.fbType),
			}, []reflect.Type{}, false),
			func(in []reflect.Value) []reflect.Value {
				return sac.onFeedback(fixedSGH, in)
			},
		).Interface(),
	})
	if err != nil {
		return err
	}

	sac.sgh.gh = gh

	return nil
}

// CancelGoal cancels the current goal.
func (sac *SimpleActionClient) CancelGoal() {
	sac.sgh.gh.Cancel()
}

// CancelAllGoals cancels all goals running on the server.
func (sac *SimpleActionClient) CancelAllGoals() {
	sac.ac.CancelAllGoals()
}

func (sac *SimpleActionClient) onTransition(
	sgh *simpleActionClientGoalHandler,
	in []reflect.Value) []reflect.Value {
	sac.mutex.Lock()
	defer sac.mutex.Unlock()

	if sgh != sac.sgh {
		return nil
	}

	gh := in[0].Interface().(*ActionClientGoalHandler)
	res := in[1]

	switchToActive := func() {
		sgh.state = simpleActionClientGoalSimpleStateActive
		if sgh.conf.OnActive != nil {
			sgh.conf.OnActive()
		}
	}

	switchToDone := func() {
		sgh.state = simpleActionClientGoalSimpleStateDone
		if sgh.conf.OnDone != nil {
			reflect.ValueOf(sgh.conf.OnDone).Call([]reflect.Value{reflect.ValueOf(sac.fullState()), res})
		}
	}

	switch gh.CommState() {
	case ActionClientCommStateActive:
		if sgh.state == simpleActionClientGoalSimpleStatePending {
			switchToActive()
		}

	case ActionClientCommStatePreempting:
		if sgh.state == simpleActionClientGoalSimpleStatePending {
			switchToActive()
		}

	case ActionClientCommStateDone:
		switch sgh.state {
		case simpleActionClientGoalSimpleStatePending,
			simpleActionClientGoalSimpleStateActive:
			switchToDone()
		}
	}

	return nil
}

func (sac *SimpleActionClient) onFeedback(
	sgh *simpleActionClientGoalHandler,
	in []reflect.Value) []reflect.Value {
	sac.mutex.Lock()
	defer sac.mutex.Unlock()

	if sgh != sac.sgh {
		return nil
	}

	if sgh.conf.OnFeedback != nil {
		reflect.ValueOf(sgh.conf.OnFeedback).Call([]reflect.Value{in[0]})
	}

	return nil
}

func (sac *SimpleActionClient) fullState() SimpleActionClientGoalState {
	switch sac.sgh.gh.CommState() {
	case ActionClientCommStateWaitingForGoalAck,
		ActionClientCommStatePending,
		ActionClientCommStateRecalling:
		return SimpleActionClientGoalStatePending

	case ActionClientCommStateActive,
		ActionClientCommStatePreempting:
		return SimpleActionClientGoalStateActive

	case ActionClientCommStateDone:
		ts, _ := sac.sgh.gh.TerminalState()
		switch ts {
		case ActionClientTerminalStateRecalled:
			return SimpleActionClientGoalStateRecalled

		case ActionClientTerminalStateRejected:
			return SimpleActionClientGoalStateRejected

		case ActionClientTerminalStatePreempted:
			return SimpleActionClientGoalStatePreempted

		case ActionClientTerminalStateAborted:
			return SimpleActionClientGoalStateAborted

		case ActionClientTerminalStateSucceeded:
			return SimpleActionClientGoalStateSucceeded

		case ActionClientTerminalStateLost:
			return SimpleActionClientGoalStateLost
		}

	case ActionClientCommStateWaitingForResult,
		ActionClientCommStateWaitingForCancelAck:
		switch sac.sgh.state {
		case simpleActionClientGoalSimpleStatePending:
			return SimpleActionClientGoalStatePending

		case simpleActionClientGoalSimpleStateActive:
			return SimpleActionClientGoalStateActive
		}
	}
	return SimpleActionClientGoalStateLost
}
