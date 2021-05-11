package goroslib

import (
	"context"
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/pkg/actionproc"
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

	ctx           context.Context
	ctxCancel     func()
	as            *ActionServer
	lastGoal      *ActionServerGoalHandler
	executingGoal *ActionServerGoalHandler

	// in
	goal chan *goalHandlerPair

	// out
	done chan struct{}
}

// NewSimpleActionServer allocates a SimpleActionServer.
func NewSimpleActionServer(conf SimpleActionServerConf) (*SimpleActionServer, error) {
	ctx, ctxCancel := context.WithCancel(context.Background())

	sas := &SimpleActionServer{
		conf:      conf,
		ctx:       ctx,
		ctxCancel: ctxCancel,
		goal:      make(chan *goalHandlerPair),
		done:      make(chan struct{}),
	}

	goal, _, _, err := actionproc.GoalResultFeedback(conf.Action)
	if err != nil {
		return nil, err
	}

	if conf.OnExecute != nil {
		cbt := reflect.TypeOf(conf.OnExecute)
		if cbt.Kind() != reflect.Func {
			return nil, fmt.Errorf("OnExecute is not a function")
		}
		if cbt.NumIn() != 2 {
			return nil, fmt.Errorf("OnExecute must accept 2 arguments")
		}
		if cbt.NumOut() != 0 {
			return nil, fmt.Errorf("OnExecute must not return any value")
		}
		if cbt.In(0) != reflect.TypeOf(&SimpleActionServer{}) {
			return nil, fmt.Errorf("OnExecute 1st argument must be %s, while is %v",
				reflect.TypeOf(&SimpleActionServer{}), cbt.In(0))
		}
		if cbt.In(1) != reflect.PtrTo(reflect.TypeOf(goal)) {
			return nil, fmt.Errorf("OnExecute 2nd argument must be %s, while is %v",
				reflect.PtrTo(reflect.TypeOf(goal)), cbt.In(1))
		}
	}

	sas.as, err = NewActionServer(ActionServerConf{
		Node:   conf.Node,
		Name:   conf.Name,
		Action: conf.Action,
		OnGoal: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{
				reflect.TypeOf(&ActionServerGoalHandler{}),
				reflect.PtrTo(reflect.TypeOf(goal)),
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
func (sas *SimpleActionServer) Close() error {
	sas.as.Close()

	go func() {
		for range sas.goal {
		}
	}()

	sas.ctxCancel()
	<-sas.done

	return nil
}

// PublishFeedback publishes a feedback about the current goal.
func (sas *SimpleActionServer) PublishFeedback(fb interface{}) {
	sas.executingGoal.PublishFeedback(fb)
}

// SetAborted sets the current goal as aborted.
func (sas *SimpleActionServer) SetAborted(res interface{}) {
	sas.executingGoal.SetAborted(res)
}

// SetSucceeded sets the current goal as succeeded.
func (sas *SimpleActionServer) SetSucceeded(res interface{}) {
	sas.executingGoal.SetSucceeded(res)
}

func (sas *SimpleActionServer) onGoal(in []reflect.Value) []reflect.Value {
	gh := in[0].Interface().(*ActionServerGoalHandler)
	goal := in[1].Interface()

	if sas.lastGoal != nil {
		sas.lastGoal.SetCanceled(reflect.New(sas.as.resType).Interface())
	}

	sas.lastGoal = gh

	gh.SetAccepted()

	sas.goal <- &goalHandlerPair{gh, goal}

	return []reflect.Value{}
}

func (sas *SimpleActionServer) onCancel(gh *ActionServerGoalHandler) {
	gh.SetCanceled(reflect.New(sas.as.resType).Interface())
}

func (sas *SimpleActionServer) run() {
	defer close(sas.done)

	executeRunning := false
	var executeDone chan struct{}

	executeLaunch := func(pair *goalHandlerPair) {
		sas.executingGoal = pair.gh
		executeRunning = true

		executeDone = make(chan struct{})
		go func() {
			defer close(executeDone)

			if sas.conf.OnExecute != nil {
				reflect.ValueOf(sas.conf.OnExecute).Call([]reflect.Value{
					reflect.ValueOf(sas),
					reflect.ValueOf(pair.goal),
				})
			}
		}()
	}

	var nextGoal *goalHandlerPair

outer:
	for {
		select {
		case pair := <-sas.goal:
			if !executeRunning {
				executeLaunch(pair)
			} else {
				nextGoal = pair
			}

		case <-executeDone:
			if nextGoal != nil {
				executeLaunch(nextGoal)
				nextGoal = nil
			}

		case <-sas.ctx.Done():
			break outer
		}
	}

	sas.ctxCancel()
}
