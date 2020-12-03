package goroslib

import (
	"fmt"
	"reflect"
	"sync"

	"github.com/aler9/goroslib/pkg/msg"
)

// Protocol is a ROS stream protocol.
type Protocol int

const (
	// TCPNoDelay is the TCPROS protocol, with the TCP_NODELAY flag set.
	TCPNoDelay Protocol = iota

	// TCP is the TCPROS protocol, without the TCP_NODELAY flag set.
	TCP

	// UDP is the UDPROS protocol.
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
	// it defaults to TCPNoDelay
	Protocol Protocol
}

// Subscriber is a ROS subscriber, an entity that can receive messages from a named channel.
type Subscriber struct {
	conf         SubscriberConf
	msgMsg       reflect.Type
	msgType      string
	msgMd5       string
	publishers   map[string]*subscriberPublisher
	publishersWg sync.WaitGroup

	// in
	publisherUpdate chan []string
	message         chan interface{}
	shutdown        chan struct{}
	terminate       chan struct{}
	nodeTerminate   chan struct{}

	// out
	done chan struct{}
}

// NewSubscriber allocates a Subscriber. See SubscriberConf for the options.
func NewSubscriber(conf SubscriberConf) (*Subscriber, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
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

	msgType, err := msg.Type(reflect.New(msgMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	msgMd5, err := msg.Md5Message(reflect.New(msgMsg.Elem()).Interface())
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
	defer close(s.done)

	cbv := reflect.ValueOf(s.conf.Callback)

outer:
	for {
		select {
		case urls := <-s.publisherUpdate:
			var addresses []string
			for _, u := range urls {
				addr, err := urlToAddress(u)
				if err != nil {
					continue
				}

				addresses = append(addresses, addr)
			}

			validPublishers := make(map[string]struct{})

			// add new publishers
			for _, addr := range addresses {
				validPublishers[addr] = struct{}{}

				if _, ok := s.publishers[addr]; !ok {
					newSubscriberPublisher(s, addr)
				}
			}

			// remove outdated publishers
			for addr, pub := range s.publishers {
				if _, ok := validPublishers[addr]; !ok {
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

	s.conf.Node.apiMasterClient.UnregisterSubscriber(
		s.conf.Node.absoluteTopicName(s.conf.Topic),
		s.conf.Node.apiSlaveServerUrl)

	for _, sp := range s.publishers {
		sp.close()
	}
	s.publishersWg.Wait()

	s.conf.Node.subscriberClose <- s

	<-s.nodeTerminate
	<-s.terminate

	close(s.publisherUpdate)
	close(s.message)
	close(s.shutdown)
}
