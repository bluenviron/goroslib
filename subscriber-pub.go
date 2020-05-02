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

	chanClose chan struct{}
	chanDone  chan struct{}
}

func newSubscriberPublisher(s *Subscriber, url string) *subscriberPublisher {
	sp := &subscriberPublisher{
		s:         s,
		url:       url,
		chanClose: make(chan struct{}),
		chanDone:  make(chan struct{}),
	}

	go sp.run()

	return sp
}

func (sp *subscriberPublisher) close() {
	sp.chanClose <- struct{}{}
	<-sp.chanDone
}

func (sp *subscriberPublisher) run() {
	defer func() { sp.chanDone <- struct{}{} }()

	host, port, err := parseUrl(sp.url)
	if err != nil {
		return
	}

	firstTime := true

	for {
		if firstTime {
			firstTime = false
		} else {
			t := time.NewTimer(5 * time.Second)
			select {
			case <-t.C:
				continue
			case <-sp.chanClose:
				return
			}
		}

		var xcs *api_slave.Client
		var err error
		chanSubDone := make(chan struct{}, 1)
		go func() {
			defer func() { chanSubDone <- struct{}{} }()
			xcs, err = api_slave.NewClient(host, port, sp.s.conf.Node.conf.Name)
		}()

		select {
		case <-chanSubDone:
		case <-sp.chanClose:
			return
		}

		if err != nil {
			continue
		}

		var proto *api_slave.TopicProtocol
		go func() {
			defer func() { chanSubDone <- struct{}{} }()
			defer xcs.Close()

			var err error
			proto, err = xcs.RequestTopic(sp.s.conf.Topic, [][]string{{"TCPROS"}})
			if err != nil {
				return
			}

			if proto.Name != "TCPROS" {
				return
			}
		}()

		select {
		case <-chanSubDone:
		case <-sp.chanClose:
			xcs.Close()
			<-chanSubDone
			return
		}

		var conn *tcpros.Conn
		go func() {
			defer func() { chanSubDone <- struct{}{} }()
			conn, err = tcpros.NewClient(proto.Host, uint16(proto.Port))
		}()

		select {
		case <-chanSubDone:
		case <-sp.chanClose:
			return
		}

		if err != nil {
			continue
		}

		go func() {
			defer func() { chanSubDone <- struct{}{} }()
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

				sp.s.chanEvents <- subscriberEventMessage{msg}
			}
		}()

		select {
		case <-chanSubDone:
		case <-sp.chanClose:
			conn.Close()
			<-chanSubDone
			return
		}
	}
}
