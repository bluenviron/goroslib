package goroslib

import (
	"fmt"
	"reflect"
	"sync"
	"time"

	"github.com/aler9/goroslib/pkg/action"
	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

// ActionServerGoalStatus is the status of the goal of an action server.
type ActionServerGoalStatus int

// standard goal statuses.
const (
	ActionServerGoalStatusPending    ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_PENDING)
	ActionServerGoalStatusActive     ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_ACTIVE)
	ActionServerGoalStatusPreempted  ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_PREEMPTED)
	ActionServerGoalStatusSucceeded  ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_SUCCEEDED)
	ActionServerGoalStatusAborted    ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_ABORTED)
	ActionServerGoalStatusRejected   ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_REJECTED)
	ActionServerGoalStatusPreempting ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_PREEMPTING)
	ActionServerGoalStatusRecalling  ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_RECALLING)
	ActionServerGoalStatusRecalled   ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_RECALLED)
	ActionServerGoalStatusLost       ActionServerGoalStatus = ActionServerGoalStatus(actionlib_msgs.GoalStatus_LOST)
)

// String implements fmt.Stringer.
func (s ActionServerGoalStatus) String() string {
	switch s {
	case ActionServerGoalStatusPending:
		return "pending"
	case ActionServerGoalStatusActive:
		return "active"
	case ActionServerGoalStatusPreempted:
		return "preempted"
	case ActionServerGoalStatusSucceeded:
		return "succeeded"
	case ActionServerGoalStatusAborted:
		return "aborted"
	case ActionServerGoalStatusRejected:
		return "rejected"
	case ActionServerGoalStatusPreempting:
		return "preempting"
	case ActionServerGoalStatusRecalling:
		return "recalling"
	case ActionServerGoalStatusRecalled:
		return "recalled"
	default:
		return "lost"
	}
}

// ActionServerGoalHandler is a goal handler of an ActionServer.
type ActionServerGoalHandler struct {
	as     *ActionServer
	id     string
	status ActionServerGoalStatus
}

// PublishFeedback publishes a feedback,
func (gh *ActionServerGoalHandler) PublishFeedback(fb interface{}) {
	if reflect.TypeOf(fb) != reflect.PtrTo(gh.as.fbType) {
		panic(fmt.Errorf("argument must be %s, while is %v",
			reflect.PtrTo(gh.as.fbType), reflect.TypeOf(fb)))
	}

	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	fbAction := reflect.New(gh.as.fbActionType)

	now := time.Now()

	header := std_msgs.Header{
		Stamp: now,
	}
	fbAction.Elem().FieldByName("Header").Set(reflect.ValueOf(header))

	status := actionlib_msgs.GoalStatus{
		GoalId: actionlib_msgs.GoalID{
			Id: gh.id,
		},
		Status: uint8(gh.status),
		Text:   "",
	}
	fbAction.Elem().FieldByName("Status").Set(reflect.ValueOf(status))

	fbAction.Elem().FieldByName("Feedback").Set(reflect.ValueOf(fb).Elem())

	gh.as.feedbackPub.Write(fbAction.Interface())
}

func (gh *ActionServerGoalHandler) publishResult(res interface{}) {
	if reflect.TypeOf(res) != reflect.PtrTo(gh.as.resType) {
		panic(fmt.Errorf("argument must be %s, while is %v",
			reflect.PtrTo(gh.as.resType), reflect.TypeOf(res)))
	}

	resAction := reflect.New(gh.as.resActionType)

	now := time.Now()

	header := std_msgs.Header{
		Stamp: now,
	}
	resAction.Elem().FieldByName("Header").Set(reflect.ValueOf(header))

	status := actionlib_msgs.GoalStatus{
		GoalId: actionlib_msgs.GoalID{
			Id: gh.id,
		},
		Status: uint8(gh.status),
		Text:   "",
	}
	resAction.Elem().FieldByName("Status").Set(reflect.ValueOf(status))

	resAction.Elem().FieldByName("Result").Set(reflect.ValueOf(res).Elem())

	gh.as.resultPub.Write(resAction.Interface())
}

// SetAccepted sets the goal as accepted.
func (gh *ActionServerGoalHandler) SetAccepted() {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()
	gh.status = ActionServerGoalStatusActive
}

// SetRejected sets the goal as rejected.
func (gh *ActionServerGoalHandler) SetRejected(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()
	gh.status = ActionServerGoalStatusRejected

	gh.publishResult(res)
}

// SetAborted sets the goal as aborted.
func (gh *ActionServerGoalHandler) SetAborted(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()
	gh.status = ActionServerGoalStatusAborted

	gh.publishResult(res)
}

// SetCanceled sets the goal as canceled.
func (gh *ActionServerGoalHandler) SetCanceled(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()
	gh.status = ActionServerGoalStatusPreempted

	gh.publishResult(res)
}

// SetSucceeded sets the goal as succeeded.
func (gh *ActionServerGoalHandler) SetSucceeded(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()
	gh.status = ActionServerGoalStatusSucceeded

	gh.publishResult(res)
}

// ActionServerConf is the configuration of an ActionServer.
type ActionServerConf struct {
	// node which the action server belongs to
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type
	Action interface{}

	// function in the form func(*ActionGoalHandler, *ActionGoal) that will be called
	// whenever a goal arrives.
	OnGoal interface{}

	// function in the form func(*ActionGoalHandler) that will be called
	// whenever a goal cancellation request arrives.
	OnCancel interface{}
}

// ActionServer is a ROS action server, an entity that can provide actions.
type ActionServer struct {
	conf           ActionServerConf
	goalType       reflect.Type
	resType        reflect.Type
	fbType         reflect.Type
	goalActionType reflect.Type
	resActionType  reflect.Type
	fbActionType   reflect.Type
	statusPub      *Publisher
	feedbackPub    *Publisher
	resultPub      *Publisher
	goalSub        *Subscriber
	cancelSub      *Subscriber
	mutex          sync.Mutex
	goals          map[string]*ActionServerGoalHandler

	// in
	terminate chan struct{}

	// out
	done chan struct{}
}

// NewActionServer allocates an ActionServer. See ActionServerConf for the options.
func NewActionServer(conf ActionServerConf) (*ActionServer, error) {
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

	as := &ActionServer{
		conf:           conf,
		goalType:       reflect.TypeOf(goal),
		resType:        reflect.TypeOf(res),
		fbType:         reflect.TypeOf(fb),
		goalActionType: reflect.TypeOf(goalAction),
		resActionType:  reflect.TypeOf(resAction),
		fbActionType:   reflect.TypeOf(fbAction),
		goals:          make(map[string]*ActionServerGoalHandler),
		terminate:      make(chan struct{}),
		done:           make(chan struct{}),
	}

	if conf.OnGoal != nil {
		cbt := reflect.TypeOf(conf.OnGoal)
		if cbt.Kind() != reflect.Func {
			return nil, fmt.Errorf("OnGoal is not a function")
		}
		if cbt.NumIn() != 2 {
			return nil, fmt.Errorf("OnGoal must accept a single argument")
		}
		if cbt.NumOut() != 0 {
			return nil, fmt.Errorf("OnGoal must not return any value")
		}
		if cbt.In(0) != reflect.TypeOf(&ActionServerGoalHandler{}) {
			return nil, fmt.Errorf("OnGoal 1st argument must be %s, while is %v",
				reflect.TypeOf(&ActionServerGoalHandler{}), cbt.In(0))
		}
		if cbt.In(1) != reflect.PtrTo(as.goalType) {
			return nil, fmt.Errorf("OnGoal 2nd argument must be %s, while is %v",
				reflect.PtrTo(as.goalType), cbt.In(1))
		}
	}

	if conf.OnCancel != nil {
		cbt := reflect.TypeOf(conf.OnCancel)
		if cbt.Kind() != reflect.Func {
			return nil, fmt.Errorf("OnCancel is not a function")
		}
		if cbt.NumIn() != 1 {
			return nil, fmt.Errorf("OnCancel must accept a single argument")
		}
		if cbt.NumOut() != 0 {
			return nil, fmt.Errorf("OnCancel must not return any value")
		}
		if cbt.In(0) != reflect.TypeOf(&ActionServerGoalHandler{}) {
			return nil, fmt.Errorf("OnCancel 1st argument must be %s, while is %v",
				reflect.TypeOf(&ActionServerGoalHandler{}), cbt.In(0))
		}
	}

	as.statusPub, err = NewPublisher(PublisherConf{
		Node:  conf.Node,
		Topic: conf.Name + "/status",
		Msg:   &actionlib_msgs.GoalStatusArray{},
	})
	if err != nil {
		return nil, err
	}

	as.feedbackPub, err = NewPublisher(PublisherConf{
		Node:  conf.Node,
		Topic: conf.Name + "/feedback",
		Msg:   reflect.New(reflect.TypeOf(fbAction)).Interface(),
	})
	if err != nil {
		as.statusPub.Close()
		return nil, err
	}

	as.resultPub, err = NewPublisher(PublisherConf{
		Node:  conf.Node,
		Topic: conf.Name + "/result",
		Msg:   reflect.New(reflect.TypeOf(resAction)).Interface(),
	})
	if err != nil {
		as.feedbackPub.Close()
		as.statusPub.Close()
		return nil, err
	}

	as.goalSub, err = NewSubscriber(SubscriberConf{
		Node:  conf.Node,
		Topic: conf.Name + "/goal",
		Callback: reflect.MakeFunc(
			reflect.FuncOf([]reflect.Type{reflect.PtrTo(reflect.TypeOf(goalAction))}, []reflect.Type{}, false),
			as.onGoal,
		).Interface(),
	})
	if err != nil {
		as.resultPub.Close()
		as.feedbackPub.Close()
		as.statusPub.Close()
		return nil, err
	}

	as.cancelSub, err = NewSubscriber(SubscriberConf{
		Node:     conf.Node,
		Topic:    conf.Name + "/cancel",
		Callback: as.onCancel,
	})
	if err != nil {
		as.goalSub.Close()
		as.resultPub.Close()
		as.feedbackPub.Close()
		as.statusPub.Close()
		return nil, err
	}

	go as.run()

	return as, nil
}

// Close closes an ActionServer and shuts down all its operations.
func (as *ActionServer) Close() error {
	close(as.terminate)
	<-as.done
	as.cancelSub.Close()
	as.goalSub.Close()
	as.resultPub.Close()
	as.feedbackPub.Close()
	as.statusPub.Close()
	return nil
}

func (as *ActionServer) run() {
	defer close(as.done)

	statusTicker := time.NewTicker(200 * time.Millisecond)
	defer statusTicker.Stop()

	curSeq := uint32(0)

	for {
		select {
		case <-statusTicker.C:
			as.statusPub.Write(&actionlib_msgs.GoalStatusArray{
				Header: std_msgs.Header{
					Seq:   curSeq,
					Stamp: time.Now(),
				},
				StatusList: func() []actionlib_msgs.GoalStatus {
					as.mutex.Lock()
					defer as.mutex.Unlock()

					var ret []actionlib_msgs.GoalStatus

					for id, g := range as.goals {
						ret = append(ret, actionlib_msgs.GoalStatus{
							GoalId: actionlib_msgs.GoalID{
								// Stamp
								Id: id,
							},
							Status: uint8(g.status),
							Text:   "",
						})
					}

					return ret
				}(),
			})
			curSeq++

		case <-as.terminate:
			return
		}
	}
}

func (as *ActionServer) onGoal(in []reflect.Value) []reflect.Value {
	msg := in[0]

	goalID := msg.Elem().FieldByName("GoalId").
		Interface().(actionlib_msgs.GoalID)
	goal := msg.Elem().FieldByName("Goal")

	gh := func() *ActionServerGoalHandler {
		as.mutex.Lock()
		defer as.mutex.Unlock()

		gh := &ActionServerGoalHandler{
			id: goalID.Id,
			as: as,
		}
		as.goals[goalID.Id] = gh

		return gh
	}()

	if as.conf.OnGoal != nil {
		reflect.ValueOf(as.conf.OnGoal).Call([]reflect.Value{
			reflect.ValueOf(gh),
			goal.Addr(),
		})
	}

	return []reflect.Value{}
}

func (as *ActionServer) onCancel(msg *actionlib_msgs.GoalID) {

	fmt.Println("CANCEL")

}
