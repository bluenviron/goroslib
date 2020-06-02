package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/api-master"
	"github.com/aler9/goroslib/msg-utils"
	"github.com/aler9/goroslib/tcpros"
)

type publisherEvent interface {
	isPublisherEvent()
}

type publisherEventClose struct {
}

func (publisherEventClose) isPublisherEvent() {}

type publisherEventSubscriberNew struct {
	client *tcpros.Conn
	header *tcpros.HeaderSubscriber
}

func (publisherEventSubscriberNew) isPublisherEvent() {}

type publisherEventSubscriberClose struct {
	sub *publisherSubscriber
}

func (publisherEventSubscriberClose) isPublisherEvent() {}

type publisherEventWrite struct {
	msg interface{}
}

func (publisherEventWrite) isPublisherEvent() {}

// PublisherConf is the configuration of a Publisher.
type PublisherConf struct {
	// node which the publisher belongs to
	Node *Node

	// name of the topic in which messages will be written
	Topic string

	// an instance of the message that will be published
	Msg interface{}

	// (optional) whether to enable latching, that consists in saving the last
	// published message and send it to any new subscriber that connects to
	// this publisher
	Latch bool
}

// Publisher is a ROS publisher, an entity that can publish messages in a named channel.
type Publisher struct {
	conf    PublisherConf
	msgType string
	msgMd5  string

	events    chan publisherEvent
	terminate chan struct{}
	nodeDone  chan struct{}
	done      chan struct{}

	subscribers map[string]*publisherSubscriber
	lastMessage interface{}
}

// NewPublisher allocates a Publisher. See PublisherConf for the options.
func NewPublisher(conf PublisherConf) (*Publisher, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if len(conf.Topic) < 1 || conf.Topic[0] != '/' {
		return nil, fmt.Errorf("Topic must begin with /")
	}

	msgt := reflect.TypeOf(conf.Msg)
	if msgt.Kind() != reflect.Ptr {
		return nil, fmt.Errorf("Msg must be a pointer")
	}
	if msgt.Elem().Kind() != reflect.Struct {
		return nil, fmt.Errorf("Msg must be a pointer to a struct")
	}

	msgType, err := msg_utils.Type(conf.Msg)
	if err != nil {
		return nil, err
	}

	msgMd5, err := msg_utils.Md5Message(conf.Msg)
	if err != nil {
		return nil, err
	}

	p := &Publisher{
		conf:        conf,
		msgType:     msgType,
		msgMd5:      msgMd5,
		events:      make(chan publisherEvent),
		terminate:   make(chan struct{}),
		nodeDone:    make(chan struct{}),
		done:        make(chan struct{}),
		subscribers: make(map[string]*publisherSubscriber),
	}

	errored := make(chan error)
	conf.Node.events <- nodeEventPublisherNew{
		pub: p,
		err: errored,
	}
	err = <-errored
	if err != nil {
		return nil, err
	}

	go p.run()

	return p, nil
}

func (p *Publisher) run() {
outer:
	for {
		rawEvt := <-p.events
		switch evt := rawEvt.(type) {
		case publisherEventSubscriberNew:
			_, ok := p.subscribers[evt.header.Callerid]
			if ok {
				evt.client.WriteHeader(&tcpros.HeaderPublisher{
					Error: ptrString(fmt.Sprintf("topic '%s' is already subscribed by '%s'",
						p.conf.Topic, evt.header.Callerid)),
				})
				evt.client.Close()
				continue
			}

			// wildcard is used by rostopic hz
			if evt.header.Md5sum != "*" && evt.header.Md5sum != p.msgMd5 {
				evt.client.WriteHeader(&tcpros.HeaderPublisher{
					Error: ptrString(fmt.Sprintf("wrong md5: expected '%s', got '%s'",
						p.msgMd5, evt.header.Md5sum)),
				})
				evt.client.Close()
				continue
			}

			err := evt.client.WriteHeader(&tcpros.HeaderPublisher{
				Callerid: ptrString(p.conf.Node.conf.Name),
				Md5sum:   ptrString(p.msgMd5),
				Topic:    ptrString(p.conf.Topic),
				Type:     ptrString(p.msgType),
				Latching: ptrInt(func() int {
					if p.conf.Latch {
						return 1
					}
					return 0
				}()),
			})
			if err != nil {
				evt.client.Close()
				continue
			}

			p.subscribers[evt.header.Callerid] = newPublisherSubscriber(p, evt.header.Callerid, evt.client)

			if p.conf.Latch && p.lastMessage != nil {
				p.subscribers[evt.header.Callerid].writeMessage(p.lastMessage)
			}

		case publisherEventSubscriberClose:
			delete(p.subscribers, evt.sub.callerid)

		case publisherEventWrite:
			if p.conf.Latch {
				p.lastMessage = evt.msg
			}

			for _, s := range p.subscribers {
				s.writeMessage(evt.msg)
			}

		case publisherEventClose:
			break outer
		}
	}

	// consume queue
	go func() {
		for range p.events {
		}
	}()

	p.conf.Node.apiMasterClient.UnregisterPublisher(api_master.RequestUnregister{
		Topic:     p.conf.Topic[1:],
		CallerUrl: p.conf.Node.apiSlaveServer.GetUrl(),
	})

	for _, c := range p.subscribers {
		c.close()
	}

	p.conf.Node.events <- nodeEventPublisherClose{p}
	<-p.nodeDone

	// wait Close()
	<-p.terminate

	close(p.events)

	close(p.done)
}

// Close closes a Publisher and shuts down all its operations.
func (p *Publisher) Close() error {
	p.events <- publisherEventClose{}
	close(p.terminate)
	<-p.done
	return nil
}

func (p *Publisher) Write(msg interface{}) {
	if reflect.TypeOf(msg) != reflect.TypeOf(p.conf.Msg) {
		panic("wrong message type")
	}

	p.events <- publisherEventWrite{msg}
}
