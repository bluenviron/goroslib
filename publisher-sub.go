package goroslib

import (
	"github.com/aler9/goroslib/tcpros"
)

type publisherSubscriber struct {
	pub      *Publisher
	callerid string
	client   *tcpros.Conn

	done chan struct{}
}

func newPublisherSubscriber(pub *Publisher, callerid string, client *tcpros.Conn) *publisherSubscriber {
	ps := &publisherSubscriber{
		pub:      pub,
		callerid: callerid,
		client:   client,
		done:     make(chan struct{}),
	}

	go ps.run()

	return ps
}

func (ps *publisherSubscriber) close() {
	ps.client.Close()
	<-ps.done
}

func (ps *publisherSubscriber) run() {
	defer close(ps.done)

outer:
	for {
		_, err := ps.client.ReadHeaderRaw()
		if err != nil {
			break outer
		}
	}

	ps.client.Close()

	ps.pub.events <- publisherEventSubscriberClose{ps}
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	ps.client.WriteMessage(msg)
}
