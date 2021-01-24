package goroslib

import (
	"bytes"
	"errors"
	"fmt"
	"net"
	"reflect"
	"strconv"
	"time"

	"github.com/aler9/goroslib/pkg/apislave"
	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
)

var errSubscriberPubTerminate = errors.New("subscriberPublisher terminated")

type subscriberPublisher struct {
	sub     *Subscriber
	address string
	udpAddr *net.UDPAddr
	udpID   uint32

	// in
	udpFrame  chan *protoudp.Frame
	terminate chan struct{}
}

func newSubscriberPublisher(sub *Subscriber, address string) {
	sp := &subscriberPublisher{
		sub:       sub,
		address:   address,
		terminate: make(chan struct{}),
	}

	sub.publishers[address] = sp

	sub.publishersWg.Add(1)
	go sp.run()
}

func (sp *subscriberPublisher) close() {
	delete(sp.sub.publishers, sp.address)
	close(sp.terminate)
}

func (sp *subscriberPublisher) run() {
	defer sp.sub.publishersWg.Done()

	for {
		ok := func() bool {
			err := sp.runInner()
			if err == errSubscriberPubTerminate {
				return false
			}

			t := time.NewTimer(5 * time.Second)
			defer t.Stop()

			select {
			case <-t.C:
				return true

			case <-sp.terminate:
				return false
			}
		}()
		if !ok {
			break
		}
	}
}

func (sp *subscriberPublisher) runInner() error {
	xcs := apislave.NewClient(sp.address, sp.sub.conf.Node.absoluteName())

	subDone := make(chan struct{}, 1)
	var res *apislave.ResponseRequestTopic
	var err error
	go func() {
		defer close(subDone)

		protocols := func() [][]interface{} {
			if sp.sub.conf.Protocol == TCP {
				return [][]interface{}{{"TCPROS"}}
			}

			nodeIP, _, _ := net.SplitHostPort(sp.sub.conf.Node.nodeAddr.String())
			return [][]interface{}{{
				"UDPROS",
				func() []byte {
					buf := bytes.NewBuffer(nil)
					protocommon.HeaderEncode(buf, &protoudp.HeaderSubscriber{
						Callerid: sp.sub.conf.Node.absoluteName(),
						Md5sum:   sp.sub.msgMd5,
						Topic:    sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
						Type:     sp.sub.msgType,
					})
					return buf.Bytes()[4:]
				}(),
				nodeIP,
				sp.sub.conf.Node.udprosServer.Port(),
				1500,
			}}
		}()
		res, err = xcs.RequestTopic(sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic), protocols)
	}()

	select {
	case <-subDone:
	case <-sp.terminate:
		<-subDone
		return errSubscriberPubTerminate
	}

	if err != nil {
		return err
	}

	if sp.sub.conf.Protocol == TCP {
		return sp.runInnerTCP(res)
	}
	return sp.runInnerUDP(res)
}

func (sp *subscriberPublisher) runInnerTCP(res *apislave.ResponseRequestTopic) error {
	if len(res.Protocol) != 3 {
		return fmt.Errorf("wrong protocol length")
	}

	protoName, ok := res.Protocol[0].(string)
	if !ok {
		return fmt.Errorf("wrong protoName")
	}

	protoHost, ok := res.Protocol[1].(string)
	if !ok {
		return fmt.Errorf("wrong protoHost")
	}

	protoPort, ok := res.Protocol[2].(int)
	if !ok {
		return fmt.Errorf("wrong protoPort")
	}

	if protoName != "TCPROS" {
		return fmt.Errorf("wrong protoName")
	}

	addr := net.JoinHostPort(protoHost, strconv.FormatInt(int64(protoPort), 10))

	subDone := make(chan struct{}, 1)
	var conn *prototcp.Conn
	var err error
	go func() {
		defer close(subDone)
		conn, err = prototcp.NewClient(addr)
	}()

	select {
	case <-subDone:
	case <-sp.terminate:
		return errSubscriberPubTerminate
	}

	if err != nil {
		return err
	}

	defer conn.Close()

	if sp.sub.conf.EnableKeepAlive {
		conn.NetConn().SetKeepAlive(true)
		conn.NetConn().SetKeepAlivePeriod(60 * time.Second)
	}

	if sp.sub.conf.DisableNoDelay {
		err := conn.NetConn().SetNoDelay(false)
		if err != nil {
			return err
		}
	}

	subDone = make(chan struct{})
	var outHeader prototcp.HeaderPublisher
	go func() {
		defer close(subDone)

		err = conn.WriteHeader(&prototcp.HeaderSubscriber{
			Callerid: sp.sub.conf.Node.absoluteName(),
			Md5sum:   sp.sub.msgMd5,
			Topic:    sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic),
			Type:     sp.sub.msgType,
			TcpNodelay: func() int {
				if sp.sub.conf.DisableNoDelay {
					return 0
				}
				return 1
			}(),
		})
		if err != nil {
			return
		}

		err = conn.ReadHeader(&outHeader)
	}()

	select {
	case <-subDone:
	case <-sp.terminate:
		conn.Close()
		<-subDone
		return errSubscriberPubTerminate
	}

	if err != nil {
		return err
	}

	if outHeader.Topic != sp.sub.conf.Node.absoluteTopicName(sp.sub.conf.Topic) {
		return fmt.Errorf("wrong topic")
	}

	if outHeader.Md5sum != sp.sub.msgMd5 {
		return fmt.Errorf("wrong md5")
	}

	if sp.sub.conf.onPublisher != nil {
		sp.sub.conf.onPublisher()
	}

	subDone = make(chan struct{})
	go func() {
		defer close(subDone)

		for {
			msg := reflect.New(sp.sub.msgMsg).Interface()
			err = conn.ReadMessage(msg)
			if err != nil {
				return
			}

			if sp.sub.conf.QueueSize == 0 {
				sp.sub.message <- msg
			} else {
				select {
				case sp.sub.message <- msg:
				default:
				}
			}
		}
	}()

	select {
	case <-subDone:
		return err

	case <-sp.terminate:
		conn.Close()
		<-subDone
		return errSubscriberPubTerminate
	}
}

func (sp *subscriberPublisher) runInnerUDP(res *apislave.ResponseRequestTopic) error {
	if len(res.Protocol) != 6 {
		return fmt.Errorf("wrong protocol length")
	}

	protoName, ok := res.Protocol[0].(string)
	if !ok {
		return fmt.Errorf("wrong protoName")
	}

	protoHost, ok := res.Protocol[1].(string)
	if !ok {
		return fmt.Errorf("wrong protoHost")
	}

	protoPort, ok := res.Protocol[2].(int)
	if !ok {
		return fmt.Errorf("wrong protoPort")
	}

	protoID, ok := res.Protocol[3].(int)
	if !ok {
		return fmt.Errorf("wrong protoID")
	}

	if protoName != "UDPROS" {
		return fmt.Errorf("wrong protoName")
	}

	// solve host and port
	addr, err := net.ResolveUDPAddr("udp", net.JoinHostPort(protoHost, strconv.FormatInt(int64(protoPort), 10)))
	if err != nil {
		return fmt.Errorf("unable to solve host")
	}

	sp.udpAddr = addr
	sp.udpID = uint32(protoID)
	sp.udpFrame = make(chan *protoudp.Frame)

	sp.sub.conf.Node.udpSubPublisherNew <- sp

	defer func() {
		done := make(chan struct{})
		sp.sub.conf.Node.udpSubPublisherClose <- udpSubPublisherCloseReq{sp, done}
		<-done
		close(sp.udpFrame)
	}()

	if sp.sub.conf.onPublisher != nil {
		sp.sub.conf.onPublisher()
	}

	readerClose := make(chan struct{})
	readerDone := make(chan struct{})
	if sp.sub.conf.EnableKeepAlive {
		go func() {
			defer close(readerDone)

			curMessageID := uint8(0)

			t := time.NewTicker(60 * time.Second)
			defer t.Stop()

			for {
				select {
				case <-t.C:
					sp.sub.conf.Node.udprosServer.WriteFrame(&protoudp.Frame{
						ConnectionID: sp.udpID,
						Opcode:       protoudp.Ping,
						MessageID:    curMessageID,
					}, sp.udpAddr)
					curMessageID++

				case <-readerClose:
					return
				}
			}
		}()
	}

	var curMsg []byte
	curFieldID := 0
	curFieldCount := 0

	for {
		select {
		case frame := <-sp.udpFrame:
			switch frame.Opcode {
			case protoudp.Data0:
				curMsg = append([]byte{}, frame.Content...)
				curFieldID = 0
				curFieldCount = int(frame.BlockID)

			case protoudp.DataN:
				if int(frame.BlockID) != (curFieldID + 1) {
					continue
				}
				curMsg = append(curMsg, frame.Content...)
				curFieldID++
			}

			if (curFieldID + 1) == curFieldCount {
				msg := reflect.New(sp.sub.msgMsg).Interface()
				err := protocommon.MessageDecode(bytes.NewBuffer(curMsg), msg)
				if err != nil {
					continue
				}

				if sp.sub.conf.QueueSize == 0 {
					sp.sub.message <- msg
				} else {
					select {
					case sp.sub.message <- msg:
					default:
					}
				}
			}

		case <-sp.terminate:
			if sp.sub.conf.EnableKeepAlive {
				close(readerClose)
				<-readerDone
			}

			return errSubscriberPubTerminate
		}
	}
}
