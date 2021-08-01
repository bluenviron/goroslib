package goroslib

import (
	"bytes"
	"context"
	"encoding/binary"
	"fmt"
	"net"
	"reflect"
	"strconv"
	"sync"

	"github.com/aler9/goroslib/pkg/apislave"
	"github.com/aler9/goroslib/pkg/msgproc"
	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
)

// PublisherConf is the configuration of a Publisher.
type PublisherConf struct {
	// parent node.
	Node *Node

	// name of the topic in which messages will be written
	Topic string

	// an instance of the message that will be published
	Msg interface{}

	// (optional) whether to enable latching, that consists in saving the last
	// published message and send it to any new subscriber that connects to
	// this publisher
	Latch bool

	onSubscriber func()
}

// Publisher is a ROS publisher, an entity that can publish messages in a named channel.
type Publisher struct {
	conf PublisherConf

	ctx           context.Context
	ctxCancel     func()
	msgType       string
	msgMd5        string
	subscribers   map[string]*publisherSubscriber
	subscribersWg sync.WaitGroup
	lastMessage   interface{}
	id            int

	// in
	getBusInfo       chan getBusInfoSubReq
	requestTopic     chan subscriberRequestTopicReq
	subscriberTCPNew chan tcpConnSubscriberReq
	subscriberClose  chan *publisherSubscriber
	write            chan interface{}

	// out
	done chan struct{}
}

// NewPublisher allocates a Publisher. See PublisherConf for the options.
func NewPublisher(conf PublisherConf) (*Publisher, error) {
	if conf.Node == nil {
		return nil, fmt.Errorf("Node is empty")
	}

	if conf.Topic == "" {
		return nil, fmt.Errorf("Topic is empty")
	}

	if conf.Msg == nil {
		return nil, fmt.Errorf("Msg is empty")
	}

	msgType, err := msgproc.Type(conf.Msg)
	if err != nil {
		return nil, err
	}

	msgMd5, err := msgproc.MD5(conf.Msg)
	if err != nil {
		return nil, err
	}

	conf.Topic = conf.Node.applyCliRemapping(conf.Topic)

	ctx, ctxCancel := context.WithCancel(conf.Node.ctx)

	p := &Publisher{
		conf:             conf,
		ctx:              ctx,
		ctxCancel:        ctxCancel,
		msgType:          msgType,
		msgMd5:           msgMd5,
		subscribers:      make(map[string]*publisherSubscriber),
		getBusInfo:       make(chan getBusInfoSubReq),
		requestTopic:     make(chan subscriberRequestTopicReq),
		subscriberTCPNew: make(chan tcpConnSubscriberReq),
		subscriberClose:  make(chan *publisherSubscriber),
		write:            make(chan interface{}),
		done:             make(chan struct{}),
	}

	p.conf.Node.Log(LogLevelDebug, "publisher '%s' created",
		p.conf.Node.absoluteTopicName(p.conf.Topic))

	cerr := make(chan error)
	select {
	case conf.Node.publisherNew <- publisherNewReq{pub: p, res: cerr}:
		err = <-cerr
		if err != nil {
			return nil, err
		}

	case <-conf.Node.ctx.Done():
		return nil, fmt.Errorf("terminated")
	}

	go p.run()

	return p, nil
}

// Close closes a Publisher and shuts down all its operations.
func (p *Publisher) Close() error {
	p.ctxCancel()
	<-p.done

	p.conf.Node.Log(LogLevelDebug, "publisher '%s' destroyed",
		p.conf.Node.absoluteTopicName(p.conf.Topic))
	return nil
}

func (p *Publisher) run() {
	defer close(p.done)

outer:
	for {
		select {
		case req := <-p.getBusInfo:
			for _, ps := range p.subscribers {
				*req.pbusInfo = append(*req.pbusInfo, ps.busInfo())
			}
			close(req.done)

		case req := <-p.requestTopic:
			err := func() error {
				if len(req.req.Protocols) < 1 {
					return fmt.Errorf("invalid protocol")
				}

				proto := req.req.Protocols[0]

				if len(proto) < 1 {
					return fmt.Errorf("invalid protocol")
				}

				protoName, ok := proto[0].(string)
				if !ok {
					return fmt.Errorf("invalid protocol")
				}

				switch protoName {
				case "TCPROS":
					nodeIP, _, _ := net.SplitHostPort(p.conf.Node.nodeAddr.String())
					req.res <- apislave.ResponseRequestTopic{
						Code:          1,
						StatusMessage: "",
						Protocol: []interface{}{
							"TCPROS",
							nodeIP,
							p.conf.Node.tcprosServer.Port(),
						},
					}
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

					raw, err := protocommon.HeaderRawDecode(buf)
					if err != nil {
						return err
					}

					var header protoudp.HeaderSubscriber
					err = protocommon.HeaderDecode(raw, &header)
					if err != nil {
						return err
					}

					_, ok = p.subscribers[header.Callerid]
					if ok {
						return fmt.Errorf("topic '%s' is already subscribed by '%s'",
							p.conf.Topic, header.Callerid)
					}

					if header.Md5sum != p.msgMd5 {
						return fmt.Errorf("wrong message checksum, expected '%s', got '%s'",
							p.msgMd5, header.Md5sum)
					}

					udpAddr, err := net.ResolveUDPAddr("udp", net.JoinHostPort(protoHost, strconv.FormatInt(int64(protoPort), 10)))
					if err != nil {
						return fmt.Errorf("unable to solve udp address)")
					}

					// if subscriber is in localhost, send packets from localhost to localhost
					// this avoids a bug in which the source ip is randomly chosen
					// from all available interfaces, making ip-based filtering unpractical
					isLocalhost := func() bool {
						ifaces, err := net.Interfaces()
						if err != nil {
							return false
						}

						for _, i := range ifaces {
							addrs, err := i.Addrs()
							if err != nil {
								continue
							}

							for _, addr := range addrs {
								if v, ok := addr.(*net.IPNet); ok {
									if v.IP.Equal(udpAddr.IP) {
										return true
									}
								}
							}
						}
						return false
					}()

					if isLocalhost {
						udpAddr.IP = net.IPv4(127, 0, 0, 1)
					}

					ps := newPublisherSubscriber(p,
						header.Callerid, nil, udpAddr)
					p.subscribers[header.Callerid] = ps

					req.res <- apislave.ResponseRequestTopic{
						Code:          1,
						StatusMessage: "",
						Protocol: []interface{}{
							"UDPROS",
							func() string {
								if isLocalhost {
									return "127.0.0.1"
								}
								nodeIP, _, _ := net.SplitHostPort(p.conf.Node.nodeAddr.String())
								return nodeIP
							}(),
							p.conf.Node.udprosServer.Port(),
							p.id,
							1500,
							func() []byte {
								var buf bytes.Buffer
								protocommon.HeaderEncode(&buf, &protoudp.HeaderPublisher{
									Callerid: p.conf.Node.absoluteName(),
									Md5sum:   p.msgMd5,
									Topic:    p.conf.Node.absoluteTopicName(p.conf.Topic),
									Type:     p.msgType,
								})
								return buf.Bytes()[4:]
							}(),
						},
					}

					return nil
				}

				return fmt.Errorf("invalid protocol")
			}()
			if err != nil {
				req.res <- apislave.ResponseRequestTopic{
					Code:          0,
					StatusMessage: err.Error(),
				}
				continue
			}

		case req := <-p.subscriberTCPNew:
			err := func() error {
				_, ok := p.subscribers[req.header.Callerid]
				if ok {
					return fmt.Errorf("topic '%s' is already subscribed by '%s'",
						p.conf.Topic, req.header.Callerid)
				}

				// wildcard is used by rostopic hz
				if req.header.Md5sum != "*" && req.header.Md5sum != p.msgMd5 {
					return fmt.Errorf("wrong message checksum, expected '%s', got '%s'",
						p.msgMd5, req.header.Md5sum)
				}

				err := req.conn.WriteHeader(&prototcp.HeaderPublisher{
					Callerid: p.conf.Node.absoluteName(),
					Md5sum:   p.msgMd5,
					Topic:    p.conf.Node.absoluteTopicName(p.conf.Topic),
					Type:     p.msgType,
					Latching: func() int {
						if p.conf.Latch {
							return 1
						}
						return 0
					}(),
				})
				if err != nil {
					req.conn.Close()
					return nil
				}

				if req.header.TcpNodelay == 0 {
					req.conn.NetConn().(*net.TCPConn).SetNoDelay(false)
				}

				ps := newPublisherSubscriber(p,
					req.header.Callerid, req.conn, nil)
				p.subscribers[req.header.Callerid] = ps

				if p.conf.Latch && p.lastMessage != nil {
					ps.writeMessage(p.lastMessage)
				}

				return nil
			}()
			if err != nil {
				p.conf.Node.Log(LogLevelError,
					"publisher '%s' is unable to accept TCP subscriber '%s': %s",
					p.conf.Node.absoluteTopicName(p.conf.Topic),
					req.conn.NetConn().RemoteAddr(),
					err)

				req.conn.WriteHeader(&prototcp.HeaderError{
					Error: err.Error(),
				})
				req.conn.Close()
				continue
			}

		case ps := <-p.subscriberClose:
			delete(p.subscribers, ps.callerID)

		case msg := <-p.write:
			if p.conf.Latch {
				p.lastMessage = msg
			}

			for _, s := range p.subscribers {
				s.writeMessage(msg)
			}

		case <-p.ctx.Done():
			break outer
		}
	}

	p.ctxCancel()

	p.conf.Node.apiMasterClient.UnregisterPublisher(
		p.conf.Node.absoluteTopicName(p.conf.Topic),
		p.conf.Node.apiSlaveServerURL)

	p.subscribersWg.Wait()

	select {
	case p.conf.Node.publisherClose <- p:
	case <-p.conf.Node.ctx.Done():
	}
}

// Write writes a message into the publisher.
func (p *Publisher) Write(msg interface{}) {
	if reflect.TypeOf(msg) != reflect.TypeOf(p.conf.Msg) {
		panic("wrong message type")
	}

	select {
	case p.write <- msg:
	case <-p.ctx.Done():
	}
}
