package goroslib

import (
	"context"
	"fmt"
	"net"
	"time"

	"github.com/aler9/goroslib/pkg/prototcp"
)

type publisherSubscriber struct {
	pub      *Publisher
	callerID string
	tcpNConn net.Conn
	tcpTConn *prototcp.Conn
	udpAddr  *net.UDPAddr

	ctx          context.Context
	ctxCancel    func()
	curMessageID uint8
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
		pub:       pub,
		callerID:  callerID,
		tcpNConn:  tcpNConn,
		tcpTConn:  tcpTConn,
		udpAddr:   udpAddr,
		ctx:       ctx,
		ctxCancel: ctxCancel,
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
		"publisher '%s' doesn't have subscriber %s anymore: %s",
		ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
		ps.subscriberLabel(),
		err)
}

func (ps *publisherSubscriber) runTCP() error {
	ps.tcpNConn.SetReadDeadline(time.Time{})

	readerErr := make(chan error)
	go func() {
		readerErr <- func() error {
			buf := make([]byte, 64)
			for {
				_, err := ps.tcpNConn.Read(buf)
				if err != nil {
					return err
				}
			}
		}()
	}()

	select {
	case err := <-readerErr:
		ps.tcpNConn.Close()
		return err

	case <-ps.ctx.Done():
		ps.tcpNConn.Close()
		<-readerErr
		return fmt.Errorf("terminated")
	}
}

func (ps *publisherSubscriber) runUDP() error {
	<-ps.ctx.Done()
	return fmt.Errorf("terminated")
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	if ps.tcpNConn != nil {
		ps.tcpNConn.SetWriteDeadline(time.Now().Add(writeTimeout))
		err := ps.tcpTConn.WriteMessage(msg)
		if err != nil {
			ps.pub.conf.Node.Log(LogLevelError,
				"publisher '%s' is unable to write to subscriber %s: %s",
				ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
				ps.subscriberLabel(),
				err)
		}
	} else {
		ps.curMessageID++

		err := ps.pub.conf.Node.udprosConn.WriteMessage(
			ps.pub.id,
			ps.curMessageID,
			msg,
			ps.udpAddr)
		if err != nil {
			ps.pub.conf.Node.Log(LogLevelError,
				"publisher '%s' is unable to write to subscriber %s: %s",
				ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
				ps.subscriberLabel(),
				err)
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
