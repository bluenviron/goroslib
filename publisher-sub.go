package goroslib

import (
	"github.com/aler9/goroslib/tcpros"
)

type publisherSubscriber struct {
	pub      *Publisher
	callerid string
	client   *tcpros.Conn

	chanDone chan struct{}
}

func newPublisherSubscriber(pub *Publisher, callerid string, client *tcpros.Conn) *publisherSubscriber {
	ps := &publisherSubscriber{
		pub:      pub,
		callerid: callerid,
		client:   client,
		chanDone: make(chan struct{}),
	}

	go ps.run()

	return ps
}

func (ps *publisherSubscriber) close() {
	ps.client.Close()
	<-ps.chanDone
}

func (ps *publisherSubscriber) run() {
	defer func() { ps.chanDone <- struct{}{} }()

outer:
	for {
		_, err := ps.client.ReadHeaderRaw()
		if err != nil {
			break outer
		}
	}

	ps.client.Close()

	ps.pub.chanEvents <- publisherEventSubscriberClose{ps}
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	ps.client.WriteMessage(msg)
}
