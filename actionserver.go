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

type actionServerGoalHandler struct {
	//status uint8
}

// ActionServerConf is the configuration of an ActionServer.
type ActionServerConf struct {
	// node which the action server belongs to
	Node *Node

	// name of the action.
	Name string

	// an instance of the action type
	Action interface{}

	OnGoal interface{}

	OnCancel interface{}
}

// ActionServer is a ROS action server, an entity that can provide actions.
type ActionServer struct {
	conf        ActionServerConf
	statusPub   *Publisher
	feedbackPub *Publisher
	resultPub   *Publisher
	goalSub     *Subscriber
	cancelSub   *Subscriber
	wg          sync.WaitGroup
	mutex       sync.Mutex
	goals       map[string]*actionServerGoalHandler

	// in
	terminate chan struct{}
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

	goalAction, resAction, fbAction, err := action.Messages(conf.Action)
	if err != nil {
		return nil, err
	}

	as := &ActionServer{
		conf:      conf,
		goals:     make(map[string]*actionServerGoalHandler),
		terminate: make(chan struct{}),
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
			func(in []reflect.Value) []reflect.Value {
				msg := in[0].Interface()

				fmt.Printf("GOAL %+v\n", msg)

				return []reflect.Value{}
			}).Interface(),
	})
	if err != nil {
		as.resultPub.Close()
		as.feedbackPub.Close()
		as.statusPub.Close()
		return nil, err
	}

	as.cancelSub, err = NewSubscriber(SubscriberConf{
		Node:  conf.Node,
		Topic: conf.Name + "/cancel",
		Callback: func(msg *actionlib_msgs.GoalID) {
			fmt.Println("CANCEL")
		},
	})
	if err != nil {
		as.goalSub.Close()
		as.resultPub.Close()
		as.feedbackPub.Close()
		as.statusPub.Close()
		return nil, err
	}

	as.wg.Add(1)
	go as.run()

	return as, nil
}

// Close closes an ActionServer and shuts down all its operations.
func (as *ActionServer) Close() error {
	close(as.terminate)
	as.wg.Wait()
	as.cancelSub.Close()
	as.goalSub.Close()
	as.resultPub.Close()
	as.feedbackPub.Close()
	as.statusPub.Close()
	return nil
}

func (as *ActionServer) run() {
	defer as.wg.Done()

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

					//for _, sta := range

					// asdasd
					/*
						GoalId          GoalID //nolint:golint
						Status          uint8  //nolint:golint
						Text            string //nolint:golint
					*/

					return ret
				}(),
			})
			curSeq++

		case <-as.terminate:
			return
		}
	}
}
