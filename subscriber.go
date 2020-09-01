package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/api-master"
	"github.com/aler9/goroslib/msg-utils"
)

// Protocol is a ROS stream protocol.
type Protocol int

const (
	// TCP is the TCPROS protocol
	TCP Protocol = iota

	// UDP is the UDPROS protocol
	UDP
)

// SubscriberConf is the configuration of a Subscriber.
type SubscriberConf struct {
	// node which the subscriber belongs to
	Node *Node

	// name of the topic from which messages will be read
	Topic string

	// function in the form func(msg *NameOfMessage){} that will be called
	// whenever a message arrives
	Callback interface{}

	// (optional) protocol that will be used to receive messages
	// it defaults to TCPROS
	Protocol Protocol
}

// Subscriber is a ROS subscriber, an entity that can receive messages from a named channel.
type Subscriber struct {
	conf       SubscriberConf
	msgMsg     reflect.Type
	msgType    string
	msgMd5     string
	publishers map[string]*subscriberPublisher

	publisherUpdate chan []string
	message         chan interface{}
	shutdown        chan struct{}
	terminate       chan struct{}
	nodeTerminate   chan struct{}
	done            chan struct{}
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

	msgType, err := msg_utils.Type(reflect.New(msgMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	msgMd5, err := msg_utils.Md5Message(reflect.New(msgMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	s := &Subscriber{
		conf:            conf,
		msgMsg:          msgMsg.Elem(),
		msgType:         msgType,
		msgMd5:          msgMd5,
		publishers:      make(map[string]*subscriberPublisher),
		publisherUpdate: make(chan []string),
		message:         make(chan interface{}),
		shutdown:        make(chan struct{}),
		terminate:       make(chan struct{}),
		nodeTerminate:   make(chan struct{}),
		done:            make(chan struct{}),
	}

	chanErr := make(chan error)
	conf.Node.subscriberNew <- subscriberNewReq{
		sub: s,
		err: chanErr,
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
	s.shutdown <- struct{}{}
	close(s.terminate)
	<-s.done
	return nil
}

func (s *Subscriber) run() {
	cbv := reflect.ValueOf(s.conf.Callback)

outer:
	for {
		select {
		case urls := <-s.publisherUpdate:
			validPublishers := make(map[string]struct{})

			// add new publishers
			for _, url := range urls {
				validPublishers[url] = struct{}{}

				if _, ok := s.publishers[url]; !ok {
					s.publishers[url] = newSubscriberPublisher(s, url)
				}
			}

			// remove outdated publishers
			for url, pub := range s.publishers {
				if _, ok := validPublishers[url]; !ok {
					pub.close()
				}
			}

		// messages are received through events
		// in order to avoid mutexes in the user side
		case msg := <-s.message:
			cbv.Call([]reflect.Value{reflect.ValueOf(msg)})

		case <-s.shutdown:
			break outer
		}
	}

	go func() {
		for {
			select {
			case _, ok := <-s.publisherUpdate:
				if !ok {
					return
				}

			case <-s.message:
			case <-s.shutdown:
			}
		}
	}()

	s.conf.Node.apiMasterClient.UnregisterSubscriber(api_master.RequestUnregister{
		Topic:     s.conf.Topic[1:],
		CallerUrl: s.conf.Node.apiSlaveServerUrl,
	})

	for _, sp := range s.publishers {
		sp.close()
		<-sp.done
	}

	s.conf.Node.subscriberClose <- s

	<-s.nodeTerminate
	<-s.terminate

	close(s.publisherUpdate)
	close(s.message)
	close(s.shutdown)
	close(s.done)
}
