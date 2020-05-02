package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/msg-utils"
)

type subscriberEvent interface {
}

type subscriberEventClose struct {
}

type subscriberEventPublisherUpdate struct {
	urls []string
}

type subscriberEventMessage struct {
	msg interface{}
}

// SubscriberConf is the configuration of a Subscriber.
type SubscriberConf struct {
	// node which the subscriber belongs to
	Node *Node

	// name of the topic from which messages will be read
	Topic string

	// function in the form func(msg *NameOfMessage){} that will be called
	// whenever a message arrives
	Callback interface{}
}

// Subscriber is a ROS subscriber, an entity that can receive messages from a named channel.
type Subscriber struct {
	conf    SubscriberConf
	msgMsg  reflect.Type
	msgType string
	msgMd5  string

	chanEvents chan subscriberEvent
	chanDone   chan struct{}

	publishers map[string]*subscriberPublisher
}

// NewSubscriber allocates a Subscriber. See SubscriberConf for the options.
func NewSubscriber(conf SubscriberConf) (*Subscriber, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if len(conf.Topic) < 1 || conf.Topic[0] != '/' {
		return nil, fmt.Errorf("Topic must begin with /")
	}

	cbt := reflect.TypeOf(conf.Callback)
	if cbt.Kind() != reflect.Func {
		return nil, fmt.Errorf("Callback is not a function")
	}
	if cbt.NumIn() != 1 {
		return nil, fmt.Errorf("Callback must accept a single argument")
	}
	if cbt.NumOut() != 0 {
		return nil, fmt.Errorf("Callback must not return any value")
	}

	msgMsg := cbt.In(0)
	if msgMsg.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Message must be a pointer")
	}
	if msgMsg.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Message must be a pointer to a struct")
	}

	msgType, err := msg_utils.MessageType(reflect.New(msgMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	msgMd5, err := msg_utils.MessageMd5(reflect.New(msgMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	s := &Subscriber{
		conf:       conf,
		msgMsg:     msgMsg.Elem(),
		msgType:    msgType,
		msgMd5:     msgMd5,
		chanEvents: make(chan subscriberEvent),
		chanDone:   make(chan struct{}),
		publishers: make(map[string]*subscriberPublisher),
	}

	chanErr := make(chan error)
	conf.Node.chanEvents <- nodeEventSubscriberNew{
		sub:     s,
		chanErr: chanErr,
	}
	err = <-chanErr
	if err != nil {
		return nil, err
	}

	go s.run()

	return s, nil
}

// Close closes a Subscriber and shuts down all its operations.
func (s *Subscriber) Close() error {
	s.chanEvents <- subscriberEventClose{}
	<-s.chanDone
	return nil
}

func (s *Subscriber) run() {
	defer func() { s.chanDone <- struct{}{} }()

	cbv := reflect.ValueOf(s.conf.Callback)

outer:
	for {
		rawEvt := <-s.chanEvents
		switch evt := rawEvt.(type) {
		case subscriberEventClose:
			break outer

		case subscriberEventPublisherUpdate:
			validPublishers := make(map[string]struct{})

			// add new publishers
			for _, url := range evt.urls {
				if _, ok := s.publishers[url]; !ok {
					validPublishers[url] = struct{}{}
					s.publishers[url] = newSubscriberPublisher(s, url)
				}
			}

			// remove outdated publishers
			for url, pub := range s.publishers {
				if _, ok := validPublishers[url]; !ok {
					pub.close()
					delete(s.publishers, url)
				}
			}

		// messages are received through events
		// in order to avoid mutexes in the user side
		case subscriberEventMessage:
			cbv.Call([]reflect.Value{reflect.ValueOf(evt.msg)})
		}
	}

	// consume queue
	go func() {
		for range s.chanEvents {
		}
	}()

	for _, pub := range s.publishers {
		pub.close()
	}

	s.conf.Node.chanEvents <- nodeEventSubscriberClose{s}

	close(s.chanEvents)
}
