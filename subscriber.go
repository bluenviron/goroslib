package goroslib

import (
	"fmt"
	"net/url"
	"reflect"
	"sync"

	"github.com/aler9/goroslib/pkg/msg"
)

// Protocol is a ROS stream protocol.
type Protocol int

const (
	// TCP is the TCPROS protocol.
	TCP Protocol = iota

	// UDP is the UDPROS protocol.
	UDP
)

// SubscriberConf is the configuration of a Subscriber.
type SubscriberConf struct {
	// node which the subscriber belongs to
	Node *Node

	// name of the topic from which messages will be read
	Topic string

	// function in the form func(msg *NameOfMessage) that will be called
	// whenever a message arrives.
	Callback interface{}

	// (optional) protocol that will be used to receive messages
	// it defaults to TCP.
	Protocol Protocol

	// (optional) enable keep-alive packets, that are
	// useful when there's a firewall between nodes.
	EnableKeepAlive bool

	// (optional) if protocol is TCP, disables the TCP_NODELAY flag, which
	// is enabled by default.
	// It defaults to false.
	DisableNoDelay bool

	onPublisher func()
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
	getBusInfo          chan getBusInfoSubReq
	subscriberPubUpdate chan []string
	message             chan interface{}
	shutdown            chan struct{}
	terminate           chan struct{}
	nodeTerminate       chan struct{}

	// out
	done chan struct{}
}

// NewSubscriber allocates a Subscriber. See SubscriberConf for the options.
func NewSubscriber(conf SubscriberConf) (*Subscriber, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if conf.Topic == "" {
		return nil, fmt.Errorf("Topic is empty")
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

	msgMd5, err := msg.MD5(reflect.New(msgMsg.Elem()).Interface())
	if err != nil {
		return nil, err
	}

	s := &Subscriber{
		conf:                conf,
		msgMsg:              msgMsg.Elem(),
		msgType:             msgType,
		msgMd5:              msgMd5,
		publishers:          make(map[string]*subscriberPublisher),
		getBusInfo:          make(chan getBusInfoSubReq),
		subscriberPubUpdate: make(chan []string),
		message:             make(chan interface{}),
		shutdown:            make(chan struct{}),
		terminate:           make(chan struct{}),
		nodeTerminate:       make(chan struct{}),
		done:                make(chan struct{}),
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
		case req := <-s.getBusInfo:
			proto := func() string {
				if s.conf.Protocol == UDP {
					return "UDPROS"
				}
				return "TCPROS"
			}()

			for _, ps := range s.publishers {
				ur := (&url.URL{
					Scheme: "http",
					Host:   ps.address,
					Path:   "/",
				}).String()
				*req.pbusInfo = append(*req.pbusInfo,
					[]interface{}{0, ur, "i", proto,
						s.conf.Node.absoluteTopicName(s.conf.Topic), true})
			}
			close(req.done)

		case urls := <-s.subscriberPubUpdate:
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

		// messages are received with events
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
			case req, ok := <-s.getBusInfo:
				if !ok {
					return
				}
				close(req.done)

			case _, ok := <-s.subscriberPubUpdate:
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
		s.conf.Node.apiSlaveServerURL)

	for _, sp := range s.publishers {
		sp.close()
	}
	s.publishersWg.Wait()

	s.conf.Node.subscriberClose <- s

	<-s.nodeTerminate
	<-s.terminate

	close(s.getBusInfo)
	close(s.subscriberPubUpdate)
	close(s.message)
	close(s.shutdown)
}
