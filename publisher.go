package goroslib

import (
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/msg"
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
}

type Publisher struct {
	conf   PublisherConf
	msgMd5 string

	chanEvents chan publisherEvent
	chanDone   chan struct{}

	subscribers map[string]*publisherSubscriber
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

	msgMd5, err := msg.MessageMd5(conf.Msg)
	if err != nil {
		return nil, err
	}

	p := &Publisher{
		conf:        conf,
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
				evt.client.Close()
				continue
			}

			if evt.header.Md5sum != p.msgMd5 {
				evt.client.Close()
				continue
			}

			err := evt.client.WriteHeader(&tcpros.HeaderPublisher{
				Callerid:          p.conf.Node.conf.Name,
				Md5sum:            p.msgMd5,
				Topic:             p.conf.Topic,
				Type:              "goroslib/Msg",
				Latching:          0,
				MessageDefinition: "",
			})
			if err != nil {
				evt.client.Close()
				continue
			}

			p.subscribers[evt.header.Callerid] = newPublisherSubscriber(p, evt.header.Callerid, evt.client)

		case publisherEventSubscriberClose:
			delete(p.subscribers, evt.sub.callerid)

		case publisherEventWrite:
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
