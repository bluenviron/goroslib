package goroslib

import (
	"reflect"
	"time"

	"github.com/aler9/goroslib/api-slave"
	"github.com/aler9/goroslib/tcpros"
)

type subscriberPublisher struct {
	s   *Subscriber
	url string

	terminate chan struct{}
	done      chan struct{}
}

func newSubscriberPublisher(s *Subscriber, url string) *subscriberPublisher {
	sp := &subscriberPublisher{
		s:         s,
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
				continue
			case <-sp.terminate:
				break outer
			}
		}

		xcs := api_slave.NewClient(host, port, sp.s.conf.Node.conf.Name)

		subDone := make(chan struct{})
		var protocol []interface{}
		var err error
		go func() {
			defer close(subDone)
			protocol, err = xcs.RequestTopic(api_slave.RequestRequestTopic{
				Topic:     sp.s.conf.Topic,
				Protocols: [][]string{{"TCPROS"}},
			})
		}()

		select {
		case <-subDone:
		case <-sp.terminate:
			<-subDone
			break outer
		}

		if err != nil {
			return
		}

		if len(protocol) != 3 {
			return
		}

		protoName, ok := protocol[0].(string)
		if !ok {
			return
		}

		protoHost, ok := protocol[1].(string)
		if !ok {
			return
		}

		protoPort, ok := protocol[2].(int)
		if !ok {
			return
		}

		if protoName != "TCPROS" {
			return
		}

		subDone = make(chan struct{})
		var conn *tcpros.Conn
		go func() {
			defer close(subDone)
			conn, err = tcpros.NewClient(protoHost, uint16(protoPort))
		}()

		select {
		case <-subDone:
		case <-sp.terminate:
			break outer
		}

		if err != nil {
			continue
		}

		subDone = make(chan struct{})
		go func() {
			defer close(subDone)
			defer conn.Close()

			err = conn.WriteHeader(&tcpros.HeaderSubscriber{
				Callerid:   sp.s.conf.Node.conf.Name,
				Topic:      sp.s.conf.Topic,
				Md5sum:     sp.s.msgMd5,
				Type:       sp.s.msgType,
				TcpNodelay: 0,
			})
			if err != nil {
				return
			}

			var outHeader tcpros.HeaderPublisher
			err = conn.ReadHeader(&outHeader)
			if err != nil {
				return
			}

			if outHeader.Error != nil {
				return
			}

			if outHeader.Topic == nil || outHeader.Md5sum == nil || outHeader.Latching == nil {
				return
			}

			if *outHeader.Topic != sp.s.conf.Topic {
				return
			}

			if *outHeader.Md5sum != sp.s.msgMd5 {
				return
			}

			for {
				msg := reflect.New(sp.s.msgMsg).Interface()
				err := conn.ReadMessage(msg)
				if err != nil {
					return
				}

				sp.s.events <- subscriberEventMessage{msg}
			}
		}()

		select {
		case <-subDone:
		case <-sp.terminate:
			conn.Close()
			<-subDone
			break outer
		}
	}

	close(sp.done)
}

func (sp *subscriberPublisher) close() {
	close(sp.terminate)
	<-sp.done
}
