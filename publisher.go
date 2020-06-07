package goroslib

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"reflect"

	"github.com/aler9/goroslib/api-master"
	"github.com/aler9/goroslib/api-slave"
	"github.com/aler9/goroslib/msg-utils"
	"github.com/aler9/goroslib/proto-common"
	"github.com/aler9/goroslib/proto-tcp"
	"github.com/aler9/goroslib/proto-udp"
)

type publisherEvent interface {
	isPublisherEvent()
}

type publisherEventClose struct{}

func (publisherEventClose) isPublisherEvent() {}

type publisherEventRequestTopic struct {
	req *api_slave.RequestRequestTopic
}

func (publisherEventRequestTopic) isPublisherEvent() {}

type publisherEventSubscriberTcpNew struct {
	client *proto_tcp.Conn
	header *proto_tcp.HeaderSubscriber
}

func (publisherEventSubscriberTcpNew) isPublisherEvent() {}

type publisherEventSubscriberTcpClose struct {
	sub *publisherSubscriber
}

func (publisherEventSubscriberTcpClose) isPublisherEvent() {}

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
	conf        PublisherConf
	msgType     string
	msgMd5      string
	subscribers map[string]*publisherSubscriber
	lastMessage interface{}
	id          int

	events    chan publisherEvent
	terminate chan struct{}
	nodeDone  chan struct{}
	done      chan struct{}
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
		subscribers: make(map[string]*publisherSubscriber),
		events:      make(chan publisherEvent),
		terminate:   make(chan struct{}),
		nodeDone:    make(chan struct{}),
		done:        make(chan struct{}),
	}

	chanErr := make(chan error)
	conf.Node.events <- nodeEventPublisherNew{
		pub: p,
		err: chanErr,
	}
	err = <-chanErr
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
		case publisherEventRequestTopic:
			err := func() error {
				if len(evt.req.Protocols) < 1 {
					return fmt.Errorf("invalid protocol")
				}

				proto := evt.req.Protocols[0]

				if len(proto) < 1 {
					return fmt.Errorf("invalid protocol")
				}

				protoName, ok := proto[0].(string)
				if !ok {
					return fmt.Errorf("invalid protocol")
				}

				switch protoName {
				case "TCPROS":
					p.conf.Node.apiSlaveServer.Write(api_slave.ResponseRequestTopic{
						Code:          1,
						StatusMessage: "",
						Protocol: []interface{}{
							"TCPROS",
							p.conf.Node.conf.Host,
							int(p.conf.Node.tcprosServer.Port()),
						},
					})
					return nil

				case "UDPROS":
					if len(proto) < 5 {
						return fmt.Errorf("invalid protocol")
					}

					protoDef, ok := proto[1].([]byte)
					if !ok {
						return fmt.Errorf("invalid protoDef")
					}

					protoHost, ok := proto[2].(string)
					if !ok {
						return fmt.Errorf("invalid protoHost")
					}

					protoPort, ok := proto[3].(int)
					if !ok {
						return fmt.Errorf("invalid protoPort")
					}

					_, ok = proto[4].(int)
					if !ok {
						return fmt.Errorf("invalid proto1500")
					}

					newProtoDef := make([]byte, 4)
					binary.LittleEndian.PutUint32(newProtoDef, uint32(len(protoDef)))
					newProtoDef = append(newProtoDef, protoDef...)
					buf := bytes.NewBuffer(newProtoDef)

					raw, err := proto_common.HeaderDecodeRaw(buf)
					if err != nil {
						return err
					}

					var header proto_udp.HeaderSubscriber
					err = proto_common.HeaderDecode(raw, &header)
					if err != nil {
						return err
					}

					_, ok = p.subscribers[header.Callerid]
					if ok {
						return fmt.Errorf("topic '%s' is already subscribed by '%s'",
							p.conf.Topic, header.Callerid)
					}

					if header.Md5sum != p.msgMd5 {
						return fmt.Errorf("wrong md5: expected '%s', got '%s'",
							p.msgMd5, header.Md5sum)
					}

					p.subscribers[header.Callerid] = newPublisherSubscriber(p,
						header.Callerid, nil, protoHost, protoPort)

					p.conf.Node.apiSlaveServer.Write(api_slave.ResponseRequestTopic{
						Code:          1,
						StatusMessage: "",
						Protocol: []interface{}{
							"UDPROS",
							p.conf.Node.conf.Host,
							int(p.conf.Node.udprosServer.Port()),
							p.id,
							1500,
							func() []byte {
								buf := bytes.NewBuffer(nil)
								proto_common.HeaderEncode(buf, &proto_udp.HeaderPublisher{
									Callerid: p.conf.Node.conf.Name,
									Md5sum:   p.msgMd5,
									Topic:    p.conf.Topic,
									Type:     p.msgType,
								})
								return buf.Bytes()[4:]
							}(),
						},
					})

					return nil
				}

				return fmt.Errorf("invalid protocol")
			}()
			if err != nil {
				p.conf.Node.apiSlaveServer.Write(api_slave.ResponseRequestTopic{
					Code:          0,
					StatusMessage: err.Error(),
				})
				continue
			}

		case publisherEventSubscriberTcpNew:
			err := func() error {
				_, ok := p.subscribers[evt.header.Callerid]
				if ok {
					return fmt.Errorf("topic '%s' is already subscribed by '%s'",
						p.conf.Topic, evt.header.Callerid)
				}

				// wildcard is used by rostopic hz
				if evt.header.Md5sum != "*" && evt.header.Md5sum != p.msgMd5 {
					return fmt.Errorf("wrong md5: expected '%s', got '%s'",
						p.msgMd5, evt.header.Md5sum)
				}

				err := evt.client.WriteHeader(&proto_tcp.HeaderPublisher{
					Callerid: p.conf.Node.conf.Name,
					Md5sum:   p.msgMd5,
					Topic:    p.conf.Topic,
					Type:     p.msgType,
					Latching: func() int {
						if p.conf.Latch {
							return 1
						}
						return 0
					}(),
				})
				if err != nil {
					evt.client.Close()
					return nil
				}

				p.subscribers[evt.header.Callerid] = newPublisherSubscriber(p,
					evt.header.Callerid, evt.client, "", 0)

				if p.conf.Latch && p.lastMessage != nil {
					p.subscribers[evt.header.Callerid].writeMessage(p.lastMessage)
				}

				return nil
			}()
			if err != nil {
				evt.client.WriteHeader(&proto_tcp.HeaderError{
					Error: err.Error(),
				})
				evt.client.Close()
				continue
			}

		case publisherEventSubscriberTcpClose:
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
