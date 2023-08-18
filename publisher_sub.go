package goroslib

import (
	"context"
	"fmt"
	"net"
	"time"

	"github.com/bluenviron/goroslib/v2/pkg/prototcp"
)

const (
	writeBufferSize = 64
)

type publisherSubscriber struct {
	pub      *Publisher
	callerID string
	tcpNConn net.Conn
	tcpTConn *prototcp.Conn
	udpAddr  *net.UDPAddr

	ctx       context.Context
	ctxCancel func()

	// in
	chWriteMessage chan interface{}
}

func newPublisherSubscriber(
	pub *Publisher,
	callerID string,
	tcpNConn net.Conn,
	tcpTConn *prototcp.Conn,
	udpAddr *net.UDPAddr,
) *publisherSubscriber {
	ctx, ctxCancel := context.WithCancel(pub.ctx)

	ps := &publisherSubscriber{
		pub:            pub,
		callerID:       callerID,
		tcpNConn:       tcpNConn,
		tcpTConn:       tcpTConn,
		udpAddr:        udpAddr,
		ctx:            ctx,
		ctxCancel:      ctxCancel,
		chWriteMessage: make(chan interface{}, writeBufferSize),
	}

	pub.subscribersWg.Add(1)
	go ps.run()

	return ps
}

func (ps *publisherSubscriber) subscriberLabel() string {
	if ps.tcpNConn != nil {
		return "'" + ps.tcpNConn.RemoteAddr().String() + "' (TCP)"
	}
	return "'" + ps.udpAddr.String() + "' (UDP)"
}

func (ps *publisherSubscriber) run() {
	defer ps.pub.subscribersWg.Done()

	ps.pub.conf.Node.Log(LogLevelDebug,
		"publisher '%s' got a new subscriber %s",
		ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
		ps.subscriberLabel())

	if ps.pub.conf.onSubscriber != nil {
		ps.pub.conf.onSubscriber()
	}

	var err error
	if ps.tcpNConn != nil {
		err = ps.runTCP()
	} else {
		err = ps.runUDP()
	}

	ps.ctxCancel()

	select {
	case ps.pub.subscriberClose <- ps:
	case <-ps.pub.ctx.Done():
	}

	ps.pub.conf.Node.Log(LogLevelDebug,
		"publisher '%s' lost subscriber %s: %s",
		ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
		ps.subscriberLabel(),
		err)
}

func (ps *publisherSubscriber) runTCP() error {
	ps.tcpNConn.SetReadDeadline(time.Time{})

	readerErr := make(chan error)

	go func() {
		readerErr <- ps.runTCPReader()
	}()

	writerErr := make(chan error)
	writerClose := make(chan struct{})

	go func() {
		writerErr <- ps.runTCPWriter(writerClose)
	}()

	select {
	case err := <-readerErr:
		ps.tcpNConn.Close()
		close(writerClose)
		<-writerErr
		return err

	case err := <-writerErr:
		ps.tcpNConn.Close()
		<-readerErr
		return err

	case <-ps.ctx.Done():
		ps.tcpNConn.Close()
		<-readerErr
		close(writerClose)
		<-writerErr
		return fmt.Errorf("terminated")
	}
}

func (ps *publisherSubscriber) runTCPReader() error {
	buf := make([]byte, 64)
	for {
		_, err := ps.tcpNConn.Read(buf)
		if err != nil {
			return err
		}
	}
}

func (ps *publisherSubscriber) runTCPWriter(writerClose chan struct{}) error {
	for {
		select {
		case msg := <-ps.chWriteMessage:
			ps.tcpNConn.SetWriteDeadline(time.Now().Add(ps.pub.conf.Node.conf.WriteTimeout))
			err := ps.tcpTConn.WriteMessage(msg)
			if err != nil {
				return err
			}

		case <-writerClose:
			return fmt.Errorf("terminated")
		}
	}
}

func (ps *publisherSubscriber) runUDP() error {
	curMessageID := uint8(0)

	for {
		select {
		case msg := <-ps.chWriteMessage:
			curMessageID++

			err := ps.pub.conf.Node.udprosConn.WriteMessage(
				ps.pub.id,
				curMessageID,
				msg,
				ps.udpAddr)
			if err != nil {
				return err
			}

		case <-ps.ctx.Done():
			return fmt.Errorf("terminated")
		}
	}
}

func (ps *publisherSubscriber) busInfo() []interface{} {
	proto := func() string {
		if ps.tcpNConn != nil {
			return "TCPROS"
		}
		return "UDPROS"
	}()

	return []interface{}{
		0,
		ps.callerID,
		"o",
		proto,
		ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
		true,
	}
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	select {
	case ps.chWriteMessage <- msg:
	case <-ps.ctx.Done():
	default: // buffer is full
	}
}
