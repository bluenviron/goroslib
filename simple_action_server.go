package goroslib

import (
	"context"
	"fmt"
	"reflect"
	"sync"

	"github.com/bluenviron/goroslib/v2/pkg/actionproc"
)

type goalHandlerPair struct {
	gh   *ActionServerGoalHandler
	goal interface{}
}

// SimpleActionServerConf is the configuration of a SimpleActionServer.
type SimpleActionServerConf struct {
	// parent node.
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type.
	Action interface{}

	// (optional) function in the form func(*ActionGoal) that will be
	// called when a goal arrives.
	OnExecute interface{}
}

// SimpleActionServer is a ROS simple action server, an entity that can provide actions.
type SimpleActionServer struct {
	conf SimpleActionServerConf

	ctx         context.Context
	ctxCancel   func()
	as          *ActionServer
	currentGoal *ActionServerGoalHandler
	preemptFlag bool
	mutex       sync.Mutex

	// in
	goal   chan goalHandlerPair
	cancel chan *ActionServerGoalHandler

	// out
	done chan struct{}
}

// NewSimpleActionServer allocates a SimpleActionServer.
func NewSimpleActionServer(conf SimpleActionServerConf) (*SimpleActionServer, error) {
	if reflect.TypeOf(conf.Action).Kind() != reflect.Ptr {
		return nil, fmt.Errorf("'Action' is not a pointer")
	}

	actionElem := reflect.ValueOf(conf.Action).Elem().Interface()

	goal, _, _, err := actionproc.GoalResultFeedback(actionElem)
	if err != nil {
		return nil, err
	}

	ctx, ctxCancel := context.WithCancel(context.Background())

	sas := &SimpleActionServer{
		conf:      conf,
		ctx:       ctx,
		ctxCancel: ctxCancel,
		goal:      make(chan goalHandlerPair),
		cancel:    make(chan *ActionServerGoalHandler),
		done:      make(chan struct{}),
	}

	if conf.OnExecute != nil {
		cbt := reflect.TypeOf(conf.OnExecute)
		if cbt.Kind() != reflect.Func {
			return nil, fmt.Errorf("OnExecute is not a function")
		}

		if cbt.NumIn() != 2 {
			return nil, fmt.Errorf("OnExecute must accept 2 arguments")
		}
		if cbt.In(0) != reflect.TypeOf(&SimpleActionServer{}) {
			return nil, fmt.Errorf("OnExecute 1st argument must be %s, while is %v",
				reflect.TypeOf(&SimpleActionServer{}), cbt.In(0))
		}
		if cbt.In(1) != reflect.PointerTo(reflect.TypeOf(goal)) {
			return nil, fmt.Errorf("OnExecute 2nd argument must be %s, while is %v",
				reflect.PointerTo(reflect.TypeOf(goal)), cbt.In(1))
		}

		if cbt.NumOut() != 0 {
			return nil, fmt.Errorf("OnExecute must not return any value")
		}
	}

	sas.as, err = NewActionServer(ActionServerConf{
		Node:   conf.Node,
		Name:   conf.Name,
		Action: conf.Action,
		OnGoal: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{
				reflect.TypeOf(&ActionServerGoalHandler{}),
				reflect.PointerTo(reflect.TypeOf(goal)),
			}, []reflect.Type{}, false),
			sas.onGoal,
		).Interface(),
		OnCancel: sas.onCancel,
	})
	if err != nil {
		return nil, err
	}

	go sas.run()

	return sas, nil
}

// Close closes a SimpleActionServer.
func (sas *SimpleActionServer) Close() {
	sas.ctxCancel()
	<-sas.done
}

// IsPreemptRequested checks whether the goal has been canceled.
func (sas *SimpleActionServer) IsPreemptRequested() bool {
	sas.mutex.Lock()
	defer sas.mutex.Unlock()
	return sas.preemptFlag
}

// PublishFeedback publishes a feedback about the current goal.
// This can be called only from an OnExecute callback.
func (sas *SimpleActionServer) PublishFeedback(fb interface{}) {
	sas.currentGoal.PublishFeedback(fb)
}

// SetAborted sets the current goal as aborted.
// This can be called only from an OnExecute callback.
func (sas *SimpleActionServer) SetAborted(res interface{}) {
	sas.currentGoal.SetAborted(res)
}

// SetSucceeded sets the current goal as succeeded.
// This can be called only from an OnExecute callback.
func (sas *SimpleActionServer) SetSucceeded(res interface{}) {
	sas.currentGoal.SetSucceeded(res)
}

func (sas *SimpleActionServer) onGoal(in []reflect.Value) []reflect.Value {
	gh := in[0].Interface().(*ActionServerGoalHandler)
	goal := in[1].Interface()

	select {
	case sas.goal <- goalHandlerPair{gh, goal}:
	case <-sas.ctx.Done():
	}

	return nil
}

func (sas *SimpleActionServer) onCancel(gh *ActionServerGoalHandler) {
	select {
	case sas.cancel <- gh:
	case <-sas.ctx.Done():
	}
}

func (sas *SimpleActionServer) run() {
	defer close(sas.done)

	executeRunning := false
	var executeDone chan struct{}

	executeStart := func(goal interface{}, gh *ActionServerGoalHandler, preemptFlag bool) {
		sas.currentGoal = gh
		sas.preemptFlag = preemptFlag

		executeRunning = true
		executeDone = make(chan struct{})

		go func() {
			defer close(executeDone)

			if sas.conf.OnExecute != nil {
				reflect.ValueOf(sas.conf.OnExecute).Call([]reflect.Value{
					reflect.ValueOf(sas),
					reflect.ValueOf(goal),
				})
			}
		}()
	}

	var lastGoal *ActionServerGoalHandler
	var nextGoal *goalHandlerPair
	nextGoalPreemptFlag := false

outer:
	for {
		select {
		case pair := <-sas.goal:
			if lastGoal != nil {
				lastGoal.SetCanceled(reflect.New(sas.as.resType).Interface())
			}

			lastGoal = pair.gh

			pair.gh.SetAccepted()

			if !executeRunning {
				executeStart(pair.goal, pair.gh, false)
			} else {
				nextGoal = &pair
				nextGoalPreemptFlag = false
			}

		case gh := <-sas.cancel:
			switch gh {
			case sas.currentGoal:
				sas.mutex.Lock()
				sas.preemptFlag = true
				sas.mutex.Unlock()

			case nextGoal.gh:
				nextGoalPreemptFlag = true
			}

		case <-executeDone:
			if nextGoal != nil {
				executeStart(nextGoal.goal, nextGoal.gh, nextGoalPreemptFlag)
				nextGoal = nil
				nextGoalPreemptFlag = false
			} else {
				lastGoal = nil
			}

		case <-sas.ctx.Done():
			break outer
		}
	}

	sas.ctxCancel()

	sas.as.Close()

	if executeRunning {
		<-executeDone
	}
}
