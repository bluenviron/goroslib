package goroslib

import (
	"fmt"
	"reflect"
	"sync"
)

// SimpleActionClientConf is the configuration of a SimpleActionClient.
type SimpleActionClientConf struct {
	// node which the action client belongs to.
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type.
	Action interface{}
}

type simpleActionClientGoalHandler struct {
	conf SimpleActionClientGoalConf
}

// SimpleActionClient is a ROS simple action client, an entity that can call simple actions.
type SimpleActionClient struct {
	ac             *ActionClient
	mutex          sync.Mutex
	curGoalHandler *simpleActionClientGoalHandler
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
		if cbt.NumIn() != 1 {
			return fmt.Errorf("OnDone must accept a single argument")
		}
		if cbt.NumOut() != 0 {
			return fmt.Errorf("OnDone must not return any value")
		}
		if cbt.In(0) != reflect.PtrTo(sac.ac.resType) {
			return fmt.Errorf("OnDone 1st argument must be %s, while is %v",
				reflect.PtrTo(sac.ac.resType), cbt.In(0))
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
		if cbt.In(0) != reflect.PtrTo(sac.ac.fbType) {
			return fmt.Errorf("OnFeedback 1st argument must be %s, while is %v",
				reflect.PtrTo(sac.ac.fbType), cbt.In(0))
		}
	}

	sgh := func() *simpleActionClientGoalHandler {
		sac.mutex.Lock()
		defer sac.mutex.Unlock()
		sac.curGoalHandler = &simpleActionClientGoalHandler{
			conf: conf,
		}
		return sac.curGoalHandler
	}()

	_, err := sac.ac.SendGoal(ActionClientGoalConf{
		Goal: conf.Goal,
		OnTransition: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{
				reflect.TypeOf(&ActionClientGoalHandler{}),
				reflect.PtrTo(sac.ac.resType),
			}, []reflect.Type{}, false),
			func(in []reflect.Value) []reflect.Value {
				return sac.onTransition(sgh, in)
			},
		).Interface(),
		OnFeedback: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{
				reflect.PtrTo(sac.ac.fbType),
			}, []reflect.Type{}, false),
			func(in []reflect.Value) []reflect.Value {
				return sac.onFeedback(sgh, in)
			},
		).Interface(),
	})
	if err != nil {
		return err
	}

	return nil
}

func (sac *SimpleActionClient) onTransition(sgh *simpleActionClientGoalHandler,
	in []reflect.Value) []reflect.Value {

	ok := func() bool {
		sac.mutex.Lock()
		defer sac.mutex.Unlock()
		return sgh == sac.curGoalHandler
	}()
	if !ok {
		return []reflect.Value{}
	}

	gh := in[0].Interface().(*ActionClientGoalHandler)
	res := in[1]

	switch gh.CommState() {
	case ActionClientCommStateActive:
		if sgh.conf.OnActive != nil {
			sgh.conf.OnActive()
		}

	case ActionClientCommStateDone:
		if sgh.conf.OnDone != nil {
			reflect.ValueOf(sgh.conf.OnDone).Call([]reflect.Value{
				res,
			})
		}
	}

	return []reflect.Value{}
}

func (sac *SimpleActionClient) onFeedback(sgh *simpleActionClientGoalHandler,
	in []reflect.Value) []reflect.Value {

	ok := func() bool {
		sac.mutex.Lock()
		defer sac.mutex.Unlock()
		return sgh == sac.curGoalHandler
	}()
	if !ok {
		return []reflect.Value{}
	}

	if sgh.conf.OnFeedback != nil {
		reflect.ValueOf(sgh.conf.OnFeedback).Call([]reflect.Value{
			in[0],
		})
	}

	return []reflect.Value{}
}
