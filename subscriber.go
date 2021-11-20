package goroslib

import (
	"context"
	"fmt"
	"reflect"
	"sync"

	"github.com/aler9/goroslib/pkg/msgproc"
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
	// parent node.
	Node *Node

	// name of the topic from which messages will be read.
	Topic string

	// function in the form func(msg *NameOfMessage)
	// that will be called when a message arrives.
	Callback interface{}

	// (optional) protocol that will be used to receive messages
	// it defaults to TCP.
	Protocol Protocol

	// (optional) queue size. If the Callback is too slow, the queue fills up,
	// and newer messages are discarded.
	// It defaults to zero (wait the Callback synchronously).
	QueueSize uint

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
	conf SubscriberConf

	ctx          context.Context
	ctxCancel    func()
	msgMsg       reflect.Type
	msgType      string
	msgMd5       string
	msgDef       string
	publishers   map[string]*subscriberPublisher
	publishersWg sync.WaitGroup

	// in
	getBusInfo          chan getBusInfoSubReq
	subscriberPubUpdate chan []string
	message             chan interface{}

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
	if cbt == nil || cbt.Kind() != reflect.Func {
		return nil, fmt.Errorf("Callback is not a function")
	}

	if cbt.NumIn() != 1 {
		return nil, fmt.Errorf("Callback must accept a single argument")
	}

	if cbt.NumOut() != 0 {
		return nil, fmt.Errorf("Callback must not return any value")
	}

	if cbt.In(0).Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Msg is not a pointer")
	}

	msgElem := reflect.New(cbt.In(0).Elem()).Elem().Interface()

	msgType, err := msgproc.Type(msgElem)
	if err != nil {
		return nil, err
	}

	msgMd5, err := msgproc.MD5(msgElem)
	if err != nil {
		return nil, err
	}

	msgDef, err := msgproc.Definition(msgElem)
	if err != nil {
		return nil, err
	}

	conf.Topic = conf.Node.applyCliRemapping(conf.Topic)

	ctx, ctxCancel := context.WithCancel(conf.Node.ctx)

	s := &Subscriber{
		conf:                conf,
		ctx:                 ctx,
		ctxCancel:           ctxCancel,
		msgMsg:              cbt.In(0).Elem(),
		msgType:             msgType,
		msgMd5:              msgMd5,
		msgDef:              msgDef,
		publishers:          make(map[string]*subscriberPublisher),
		getBusInfo:          make(chan getBusInfoSubReq),
		subscriberPubUpdate: make(chan []string),
		message:             make(chan interface{}, conf.QueueSize),
		done:                make(chan struct{}),
	}

	s.conf.Node.Log(LogLevelDebug, "subscriber '%s' created",
		s.conf.Node.absoluteTopicName(s.conf.Topic))

	cerr := make(chan error)
	select {
	case conf.Node.subscriberNew <- subscriberNewReq{sub: s, res: cerr}:
		err = <-cerr
		if err != nil {
			return nil, err
		}

	case <-conf.Node.ctx.Done():
		return nil, fmt.Errorf("terminated")
	}

	go s.run()

	return s, nil
}

// Close closes a Subscriber and shuts down all its operations.
func (s *Subscriber) Close() error {
	s.ctxCancel()
	<-s.done

	s.conf.Node.Log(LogLevelDebug, "subscriber '%s' destroyed",
		s.conf.Node.absoluteTopicName(s.conf.Topic))
	return nil
}

func (s *Subscriber) run() {
	defer close(s.done)

	dispatcherDone := make(chan struct{})
	go func() {
		defer close(dispatcherDone)

		cbv := reflect.ValueOf(s.conf.Callback)

		for {
			select {
			case msg := <-s.message:
				cbv.Call([]reflect.Value{reflect.ValueOf(msg)})

			case <-s.ctx.Done():
				return
			}
		}
	}()

outer:
	for {
		select {
		case req := <-s.getBusInfo:
			for _, sp := range s.publishers {
				*req.pbusInfo = append(*req.pbusInfo, sp.busInfo())
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
					sp := newSubscriberPublisher(s, addr)
					s.publishers[addr] = sp
				}
			}

			// remove outdated publishers
			for addr, sp := range s.publishers {
				if _, ok := validPublishers[addr]; !ok {
					delete(s.publishers, sp.address)
					sp.close()
				}
			}

		case <-s.ctx.Done():
			break outer
		}
	}

	s.ctxCancel()

	s.conf.Node.apiMasterClient.UnregisterSubscriber(
		s.conf.Node.absoluteTopicName(s.conf.Topic),
		s.conf.Node.apiSlaveServer.URL())

	s.publishersWg.Wait()

	<-dispatcherDone

	select {
	case s.conf.Node.subscriberClose <- s:
	case <-s.conf.Node.ctx.Done():
	}
}
