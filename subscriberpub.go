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
	sub        *Subscriber
	url        string
	udprosIp   net.IP
	udprosPort int
	udprosId   uint32

	// in
	terminate chan struct{}
}

func newSubscriberPublisher(sub *Subscriber, url string) {
	sp := &subscriberPublisher{
		sub:       sub,
		url:       url,
		terminate: make(chan struct{}),
	}

	sub.publishers[url] = sp

	sub.publishersWg.Add(1)
	go sp.run()
}

func (sp *subscriberPublisher) close() {
	delete(sp.sub.publishers, sp.url)
	close(sp.terminate)
}

func (sp *subscriberPublisher) run() {
	defer sp.sub.publishersWg.Done()

	host, port, _ := parseUrl(sp.url)

	for {
		ok := func() bool {
			err := sp.do(host, port)
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

func (sp *subscriberPublisher) do(host string, port int) error {
	xcs := apislave.NewClient(host, port, sp.sub.conf.Node.conf.Name)

	subDone := make(chan struct{}, 1)
	var res *apislave.ResponseRequestTopic
	var err error
	go func() {
		defer close(subDone)

		protocols := func() [][]interface{} {
			if sp.sub.conf.Protocol == TCP {
				return [][]interface{}{{"TCPROS"}}
			}

			return [][]interface{}{{
				"UDPROS",
				func() []byte {
					buf := bytes.NewBuffer(nil)
					protocommon.HeaderEncode(buf, &protoudp.HeaderSubscriber{
						Callerid: sp.sub.conf.Node.conf.Name,
						Md5sum:   sp.sub.msgMd5,
						Topic:    sp.sub.conf.Topic,
						Type:     sp.sub.msgType,
					})
					return buf.Bytes()[4:]
				}(),
				sp.sub.conf.Node.nodeIp.String(),
				sp.sub.conf.Node.udprosServerPort,
				1500,
			}}
		}()
		res, err = xcs.RequestTopic(sp.sub.conf.Topic, protocols)
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

func (sp *subscriberPublisher) doTcp(res *apislave.ResponseRequestTopic) error {
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
	var conn *prototcp.Conn
	var err error
	go func() {
		defer close(subDone)
		conn, err = prototcp.NewClient(protoHost, protoPort)
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
	var outHeader prototcp.HeaderPublisher
	go func() {
		defer close(subDone)

		err = conn.WriteHeader(&prototcp.HeaderSubscriber{
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

			sp.sub.message <- msg
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

func (sp *subscriberPublisher) doUdp(res *apislave.ResponseRequestTopic) error {
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

	// solve host and port
	addr, err := net.ResolveUDPAddr("udp4", protoHost+":"+strconv.FormatInt(int64(protoPort), 10))
	if err != nil {
		return fmt.Errorf("unable to solve host")
	}

	sp.udprosIp = addr.IP
	sp.udprosPort = addr.Port
	sp.udprosId = uint32(protoId)

	chanFrame := make(chan *protoudp.Frame)
	sp.sub.conf.Node.udpSubPublisherNew <- udpSubPublisherNewReq{sp, chanFrame}

	defer func() {
		done := make(chan struct{})
		sp.sub.conf.Node.udpSubPublisherClose <- udpSubPublisherCloseReq{sp, done}
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
			case protoudp.Data0:
				curMsg = append([]byte{}, frame.Content...)
				curFieldId = 0
				curFieldCount = int(frame.BlockId)

			case protoudp.DataN:
				if int(frame.BlockId) != (curFieldId + 1) {
					continue
				}
				curMsg = append(curMsg, frame.Content...)
				curFieldId += 1
			}

			if (curFieldId + 1) == curFieldCount {
				msg := reflect.New(sp.sub.msgMsg).Interface()
				err := protocommon.MessageDecode(bytes.NewBuffer(curMsg), msg)
				if err != nil {
					continue
				}

				sp.sub.message <- msg
			}

		case <-sp.terminate:
			return errSubscriberPubTerminate
		}
	}
}
