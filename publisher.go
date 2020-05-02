package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/msg-utils"
	"github.com/aler9/goroslib/tcpros"
)

type publisherEvent interface {
}

type publisherEventClose struct {
}

type publisherEventSubscriberNew struct {
	client *tcpros.Conn
	header *tcpros.HeaderSubscriber
}

type publisherEventSubscriberClose struct {
	sub *publisherSubscriber
}

type publisherEventWrite struct {
	msg interface{}
}

type PublisherConf struct {
	Node  *Node
	Topic string
	Msg   interface{}
	Latch bool
}

type Publisher struct {
	conf    PublisherConf
	msgType string
	msgMd5  string

	chanEvents chan publisherEvent
	chanDone   chan struct{}

	subscribers map[string]*publisherSubscriber
	lastMessage interface{}
}

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

	msgType, err := msg_utils.MessageType(conf.Msg)
	if err != nil {
		return nil, err
	}

	msgMd5, err := msg_utils.MessageMd5(conf.Msg)
	if err != nil {
		return nil, err
	}

	p := &Publisher{
		conf:        conf,
		msgType:     msgType,
		msgMd5:      msgMd5,
		chanEvents:  make(chan publisherEvent),
		chanDone:    make(chan struct{}),
		subscribers: make(map[string]*publisherSubscriber),
	}

	chanErr := make(chan error)
	conf.Node.chanEvents <- nodeEventPublisherNew{
		pub:     p,
		chanErr: chanErr,
	}
	err = <-chanErr
	if err != nil {
		return nil, err
	}

	go p.run()

	return p, nil
}

func (p *Publisher) Close() error {
	p.chanEvents <- publisherEventClose{}
	<-p.chanDone
	return nil
}

func (p *Publisher) run() {
	defer func() { p.chanDone <- struct{}{} }()

outer:
	for {
		rawEvt := <-p.chanEvents
		switch evt := rawEvt.(type) {
		case publisherEventClose:
			break outer

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

			if evt.header.Md5sum != p.msgMd5 {
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
		}
	}

	// consume queue
	go func() {
		for range p.chanEvents {
		}
	}()

	for _, c := range p.subscribers {
		c.close()
	}

	p.conf.Node.chanEvents <- nodeEventPublisherClose{p}

	close(p.chanEvents)
}

func (p *Publisher) Write(msg interface{}) {
	if reflect.TypeOf(msg) != reflect.TypeOf(p.conf.Msg) {
		panic("wrong message type")
	}

	p.chanEvents <- publisherEventWrite{msg}
}
