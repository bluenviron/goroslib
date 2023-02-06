package goroslib

import (
	"context"
	"fmt"
	"reflect"
	"sync"
	"time"

	"github.com/aler9/goroslib/pkg/actionproc"
	"github.com/aler9/goroslib/pkg/msgs/actionlib_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

// ActionServerGoalState is the state of the goal of an action server.
type ActionServerGoalState int

// standard goal states.
const (
	ActionServerGoalStatePending    ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_PENDING)
	ActionServerGoalStateActive     ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_ACTIVE)
	ActionServerGoalStatePreempted  ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_PREEMPTED)
	ActionServerGoalStateSucceeded  ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_SUCCEEDED)
	ActionServerGoalStateAborted    ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_ABORTED)
	ActionServerGoalStateRejected   ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_REJECTED)
	ActionServerGoalStatePreempting ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_PREEMPTING)
	ActionServerGoalStateRecalling  ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_RECALLING)
	ActionServerGoalStateRecalled   ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_RECALLED)
	ActionServerGoalStateLost       ActionServerGoalState = ActionServerGoalState(actionlib_msgs.GoalStatus_LOST)
)

var actionServerGoalStateLabels = map[ActionServerGoalState]string{
	ActionServerGoalStatePending:    "pending",
	ActionServerGoalStateActive:     "active",
	ActionServerGoalStatePreempted:  "preempted",
	ActionServerGoalStateSucceeded:  "succeeded",
	ActionServerGoalStateAborted:    "aborted",
	ActionServerGoalStateRejected:   "rejected",
	ActionServerGoalStatePreempting: "preempting",
	ActionServerGoalStateRecalling:  "recalling",
	ActionServerGoalStateRecalled:   "recalled",
	ActionServerGoalStateLost:       "lost",
}

// String implements fmt.Stringer.
func (s ActionServerGoalState) String() string {
	if l, ok := actionServerGoalStateLabels[s]; ok {
		return l
	}
	return "unknown"
}

// ActionServerGoalHandler is a goal handler of an ActionServer.
type ActionServerGoalHandler struct {
	as      *ActionServer
	id      string
	created time.Time
	ended   *time.Time
	stamp   time.Time
	state   ActionServerGoalState
}

// PublishFeedback publishes a feedback about the goal,
func (gh *ActionServerGoalHandler) PublishFeedback(fb interface{}) {
	if reflect.TypeOf(fb) != reflect.PtrTo(gh.as.fbType) {
		panic(fmt.Errorf("argument must be %s, while is %v",
			reflect.PtrTo(gh.as.fbType), reflect.TypeOf(fb)))
	}

	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	fbAction := reflect.New(gh.as.fbActionType)

	now := time.Now()

	fbAction.Elem().FieldByName("Header").Set(reflect.ValueOf(std_msgs.Header{
		Stamp: now,
	}))
	fbAction.Elem().FieldByName("Status").Set(reflect.ValueOf(actionlib_msgs.GoalStatus{
		GoalId: actionlib_msgs.GoalID{
			Id:    gh.id,
			Stamp: gh.stamp,
		},
		Status: uint8(gh.state),
	}))
	fbAction.Elem().FieldByName("Feedback").Set(reflect.ValueOf(fb).Elem())

	gh.as.feedbackPub.Write(fbAction.Interface())
}

func (gh *ActionServerGoalHandler) publishResult(res interface{}) {
	if reflect.TypeOf(res) != reflect.PtrTo(gh.as.resType) {
		panic(fmt.Errorf("argument must be %s, while is %v",
			reflect.PtrTo(gh.as.resType), reflect.TypeOf(res)))
	}

	gh.as.conf.Node.Log(LogLevelDebug, "action server '%s' has finished goal '%s' with state '%s'",
		gh.as.conf.Node.absoluteTopicName(gh.as.conf.Name),
		gh.id,
		gh.state)

	resAction := reflect.New(gh.as.resActionType)

	now := time.Now()

	gh.ended = &now

	resAction.Elem().FieldByName("Header").Set(reflect.ValueOf(std_msgs.Header{
		Stamp: now,
	}))
	resAction.Elem().FieldByName("Status").Set(reflect.ValueOf(actionlib_msgs.GoalStatus{
		GoalId: actionlib_msgs.GoalID{
			Id:    gh.id,
			Stamp: gh.stamp,
		},
		Status: uint8(gh.state),
	}))
	resAction.Elem().FieldByName("Result").Set(reflect.ValueOf(res).Elem())

	gh.as.resultPub.Write(resAction.Interface())
}

// SetAccepted sets the goal as accepted.
func (gh *ActionServerGoalHandler) SetAccepted() {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	switch gh.state {
	case ActionServerGoalStatePending:
		gh.state = ActionServerGoalStateActive

	case ActionServerGoalStateRecalling:
		gh.state = ActionServerGoalStatePreempting
	}
}

// SetCanceled sets the goal as canceled.
func (gh *ActionServerGoalHandler) SetCanceled(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	switch gh.state {
	case ActionServerGoalStatePending,
		ActionServerGoalStateRecalling:
		gh.state = ActionServerGoalStateRecalled
		gh.publishResult(res)

	case ActionServerGoalStateActive,
		ActionServerGoalStatePreempting:
		gh.state = ActionServerGoalStatePreempted
		gh.publishResult(res)
	}
}

// SetRejected sets the goal as rejected.
func (gh *ActionServerGoalHandler) SetRejected(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	switch gh.state {
	case ActionServerGoalStatePending,
		ActionServerGoalStateRecalling:
		gh.state = ActionServerGoalStateRejected
		gh.publishResult(res)
	}
}

// SetAborted sets the goal as aborted.
func (gh *ActionServerGoalHandler) SetAborted(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	switch gh.state {
	case ActionServerGoalStatePreempting,
		ActionServerGoalStateActive:
		gh.state = ActionServerGoalStateAborted
		gh.publishResult(res)
	}
}

// SetSucceeded sets the goal as succeeded.
func (gh *ActionServerGoalHandler) SetSucceeded(res interface{}) {
	gh.as.mutex.Lock()
	defer gh.as.mutex.Unlock()

	switch gh.state {
	case ActionServerGoalStatePreempting,
		ActionServerGoalStateActive:
		gh.state = ActionServerGoalStateSucceeded
		gh.publishResult(res)
	}
}

// ActionServerConf is the configuration of an ActionServer.
type ActionServerConf struct {
	// parent node.
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type.
	Action interface{}

	// (optional) status messages are published with this period.
	// It defaults to 200 ms.
	StatusPeriod time.Duration

	// (optional) goals are deleted after they are finished and
	// after this duration.
	// It defaults to 5 secs.
	DeleteFinishedGoalAfter time.Duration

	// (optional) function in the form func(*ActionServerGoalHandler, *ActionGoal)
	// that will be called when a goal arrives.
	OnGoal interface{}

	// (optional) function in the form func(*ActionServerGoalHandler)
	// that will be called when a goal cancellation request arrives.
	OnCancel func(gh *ActionServerGoalHandler)
}

// ActionServer is a ROS action server, an entity that can provide actions.
type ActionServer struct {
	conf ActionServerConf

	ctx            context.Context
	ctxCancel      func()
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

	if reflect.TypeOf(conf.Action).Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Action is not a pointer")
	}

	if conf.StatusPeriod == 0 {
		conf.StatusPeriod = 200 * time.Millisecond
	}

	if conf.DeleteFinishedGoalAfter == 0 {
		conf.DeleteFinishedGoalAfter = 5 * time.Second
	}

	actionElem := reflect.ValueOf(conf.Action).Elem().Interface()

	goal, res, fb, err := actionproc.GoalResultFeedback(actionElem)
	if err != nil {
		return nil, err
	}

	// Messages can't fail if GoalResultFeedback didn't fail
	goalAction, resAction, fbAction, _ := actionproc.Messages(actionElem)

	conf.Name = conf.Node.applyCliRemapping(conf.Name)

	ctx, ctxCancel := context.WithCancel(context.Background())

	as := &ActionServer{
		conf:           conf,
		ctx:            ctx,
		ctxCancel:      ctxCancel,
		goalType:       reflect.TypeOf(goal),
		resType:        reflect.TypeOf(res),
		fbType:         reflect.TypeOf(fb),
		goalActionType: reflect.TypeOf(goalAction),
		resActionType:  reflect.TypeOf(resAction),
		fbActionType:   reflect.TypeOf(fbAction),
		goals:          make(map[string]*ActionServerGoalHandler),
		done:           make(chan struct{}),
	}

	as.conf.Node.Log(LogLevelDebug, "action server '%s' created",
		conf.Node.absoluteTopicName(conf.Name))

	if conf.OnGoal != nil {
		cbt := reflect.TypeOf(conf.OnGoal)
		if cbt.Kind() != reflect.Func {
			return nil, fmt.Errorf("OnGoal is not a function")
		}

		if cbt.NumIn() != 2 {
			return nil, fmt.Errorf("OnGoal must accept 2 arguments")
		}
		if cbt.In(0) != reflect.TypeOf(&ActionServerGoalHandler{}) {
			return nil, fmt.Errorf("OnGoal 1st argument must be %s, while is %v",
				reflect.TypeOf(&ActionServerGoalHandler{}), cbt.In(0))
		}
		if cbt.In(1) != reflect.PtrTo(as.goalType) {
			return nil, fmt.Errorf("OnGoal 2nd argument must be %s, while is %v",
				reflect.PtrTo(as.goalType), cbt.In(1))
		}

		if cbt.NumOut() != 0 {
			return nil, fmt.Errorf("OnGoal must not return any value")
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
	as.ctxCancel()
	<-as.done

	as.conf.Node.Log(LogLevelDebug, "action server '%s' destroyed",
		as.conf.Node.absoluteTopicName(as.conf.Name))
	return nil
}

func (as *ActionServer) run() {
	defer close(as.done)

	statusTicker := time.NewTicker(as.conf.StatusPeriod)
	defer statusTicker.Stop()

	curSeq := uint32(0)

outer:
	for {
		select {
		case <-statusTicker.C:
			statuses := func() []actionlib_msgs.GoalStatus {
				as.mutex.Lock()
				defer as.mutex.Unlock()

				// remove ended goals
				now := time.Now()
				for id, gh := range as.goals {
					if gh.ended != nil &&
						now.Sub(*gh.ended) >= as.conf.DeleteFinishedGoalAfter {
						delete(as.goals, id)
					}
				}

				var ret []actionlib_msgs.GoalStatus
				for id, gh := range as.goals {
					ret = append(ret, actionlib_msgs.GoalStatus{
						GoalId: actionlib_msgs.GoalID{
							Id:    id,
							Stamp: gh.stamp,
						},
						Status: uint8(gh.state),
					})
				}
				return ret
			}()

			as.statusPub.Write(&actionlib_msgs.GoalStatusArray{
				Header: std_msgs.Header{
					Seq:   curSeq,
					Stamp: time.Now(),
				},
				StatusList: statuses,
			})
			curSeq++

		case <-as.ctx.Done():
			break outer
		}
	}

	as.ctxCancel()

	as.cancelSub.Close()
	as.goalSub.Close()
	as.resultPub.Close()
	as.feedbackPub.Close()
	as.statusPub.Close()
}

func (as *ActionServer) onGoal(in []reflect.Value) []reflect.Value {
	msg := in[0]

	goalID := msg.Elem().FieldByName("GoalId").
		Interface().(actionlib_msgs.GoalID)
	goal := msg.Elem().FieldByName("Goal")

	gh := &ActionServerGoalHandler{
		as:      as,
		id:      goalID.Id,
		created: time.Now(),
		stamp:   goalID.Stamp,
	}

	func() {
		as.mutex.Lock()
		defer as.mutex.Unlock()

		as.goals[goalID.Id] = gh
	}()

	as.conf.Node.Log(LogLevelDebug, "action server '%s' has a new goal '%s'",
		as.conf.Node.absoluteTopicName(as.conf.Name),
		goalID.Id)

	if as.conf.OnGoal != nil {
		reflect.ValueOf(as.conf.OnGoal).Call([]reflect.Value{
			reflect.ValueOf(gh),
			goal.Addr(),
		})
	}

	return nil
}

func (as *ActionServer) onCancel(msg *actionlib_msgs.GoalID) {
	if msg.Id == "" { // cancel all goals
		as.conf.Node.Log(LogLevelDebug, "action server '%s' is canceling all goals",
			as.conf.Node.absoluteTopicName(as.conf.Name))

		goals := func() []*ActionServerGoalHandler {
			as.mutex.Lock()
			defer as.mutex.Unlock()

			var ret []*ActionServerGoalHandler
			for _, gh := range as.goals {
				ret = append(ret, gh)
			}
			return ret
		}()

		for _, gh := range goals {
			if as.conf.OnCancel != nil {
				as.conf.OnCancel(gh)
			}
		}
	} else { // cancel specific goal
		gh := func() *ActionServerGoalHandler {
			as.mutex.Lock()
			defer as.mutex.Unlock()

			gh, ok := as.goals[msg.Id]
			if !ok {
				return nil
			}
			return gh
		}()
		if gh == nil {
			as.conf.Node.Log(LogLevelError, "action server '%s' is unable to cancel goal '%s'",
				as.conf.Node.absoluteTopicName(as.conf.Name),
				msg.Id)
			return
		}

		as.conf.Node.Log(LogLevelDebug, "action server '%s' is canceling goal '%s'",
			as.conf.Node.absoluteTopicName(as.conf.Name),
			msg.Id)

		if as.conf.OnCancel != nil {
			as.conf.OnCancel(gh)
		}
	}
}
