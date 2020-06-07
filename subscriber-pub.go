package goroslib

import (
	"bytes"
	"errors"
	"fmt"
	"reflect"
	"time"

	"github.com/aler9/goroslib/api-slave"
	"github.com/aler9/goroslib/proto-common"
	"github.com/aler9/goroslib/proto-tcp"
	"github.com/aler9/goroslib/proto-udp"
)

var errSubscriberPubTerminate = errors.New("subscriberPublisher terminated")

type subscriberPublisher struct {
	sub        *Subscriber
	url        string
	udprosIp   string
	udprosPort int
	udprosId   uint32

	terminate chan struct{}
	done      chan struct{}
}

func newSubscriberPublisher(sub *Subscriber, url string) *subscriberPublisher {
	sp := &subscriberPublisher{
		sub:       sub,
		url:       url,
		terminate: make(chan struct{}),
		done:      make(chan struct{}),
	}

	go sp.run()

	return sp
}

func (sp *subscriberPublisher) run() {
	host, port, _ := parseUrl(sp.url)
	firstTime := true

outer:
	for {
		if firstTime {
			firstTime = false
		} else {
			t := time.NewTimer(5 * time.Second)
			select {
			case <-t.C:
			case <-sp.terminate:
				break outer
			}
		}

		err := sp.do(host, port)
		if err == errSubscriberPubTerminate {
			break outer
		}
	}

	close(sp.done)
}

func (sp *subscriberPublisher) close() {
	close(sp.terminate)
	<-sp.done
}

func (sp *subscriberPublisher) do(host string, port int) error {
	xcs := api_slave.NewClient(host, port, sp.sub.conf.Node.conf.Name)

	subDone := make(chan struct{})
	var res *api_slave.ResponseRequestTopic
	var err error
	go func() {
		defer close(subDone)
		res, err = xcs.RequestTopic(api_slave.RequestRequestTopic{
			Topic: sp.sub.conf.Topic,
			Protocols: func() [][]interface{} {
				if sp.sub.conf.Protocol == TCP {
					return [][]interface{}{{"TCPROS"}}
				}

				return [][]interface{}{{
					"UDPROS",
					func() []byte {
						buf := bytes.NewBuffer(nil)
						proto_common.HeaderEncode(buf, &proto_udp.HeaderSubscriber{
							Callerid: sp.sub.conf.Node.conf.Name,
							Md5sum:   sp.sub.msgMd5,
							Topic:    sp.sub.conf.Topic,
							Type:     sp.sub.msgType,
						})
						return buf.Bytes()[4:]
					}(),
					sp.sub.conf.Node.conf.Host,
					sp.sub.conf.Node.conf.UdprosPort,
					1500,
				}}
			}(),
		})
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
		return sp.doTcp(res)
	}
	return sp.doUdp(res)
}

func (sp *subscriberPublisher) doTcp(res *api_slave.ResponseRequestTopic) error {
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

	subDone := make(chan struct{})
	var conn *proto_tcp.Conn
	var err error
	go func() {
		defer close(subDone)
		conn, err = proto_tcp.NewClient(protoHost, protoPort)
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

	subDone = make(chan struct{})
	var outHeader proto_tcp.HeaderPublisher
	go func() {
		defer close(subDone)

		err = conn.WriteHeader(&proto_tcp.HeaderSubscriber{
			Callerid:   sp.sub.conf.Node.conf.Name,
			Md5sum:     sp.sub.msgMd5,
			Topic:      sp.sub.conf.Topic,
			Type:       sp.sub.msgType,
			TcpNodelay: 0,
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

	if outHeader.Topic != sp.sub.conf.Topic {
		return fmt.Errorf("wrong topic")
	}

	if outHeader.Md5sum != sp.sub.msgMd5 {
		return fmt.Errorf("wrong md5")
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

			sp.sub.events <- subscriberEventMessage{msg}
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

func (sp *subscriberPublisher) doUdp(res *api_slave.ResponseRequestTopic) error {
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

	protoId, ok := res.Protocol[3].(int)
	if !ok {
		return fmt.Errorf("wrong protoId")
	}

	if protoName != "UDPROS" {
		return fmt.Errorf("wrong protoName")
	}

	sp.udprosIp = protoHost
	sp.udprosPort = protoPort
	sp.udprosId = uint32(protoId)

	chanFrame := make(chan *proto_udp.Frame)
	sp.sub.conf.Node.events <- nodeEventUdpSubPublisherNew{sp, chanFrame}

	defer func() {
		done := make(chan struct{})
		sp.sub.conf.Node.events <- nodeEventUdpSubPublisherClose{sp, done}
		<-done
		close(chanFrame)
	}()

	var curMsg []byte
	curFieldId := 0
	curFieldCount := 0

	for {
		select {
		case frame := <-chanFrame:
			switch frame.Opcode {
			case proto_udp.Data0:
				curMsg = append([]byte{}, frame.Content...)
				curFieldId = 0
				curFieldCount = int(frame.BlockId)

			case proto_udp.DataN:
				if int(frame.BlockId) != (curFieldId + 1) {
					continue
				}
				curMsg = append(curMsg, frame.Content...)
				curFieldId += 1
			}

			if (curFieldId + 1) == curFieldCount {
				msg := reflect.New(sp.sub.msgMsg).Interface()
				err := proto_common.MessageDecode(bytes.NewBuffer(curMsg), msg)
				if err != nil {
					continue
				}

				sp.sub.events <- subscriberEventMessage{msg}
			}

		case <-sp.terminate:
			return errSubscriberPubTerminate
		}
	}
}
