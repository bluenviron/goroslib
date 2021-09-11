package goroslib

import (
	"bytes"
	"context"
	"net"

	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
)

type publisherSubscriber struct {
	pub      *Publisher
	callerID string
	tcpConn  *prototcp.Conn
	udpAddr  *net.UDPAddr

	ctx          context.Context
	ctxCancel    func()
	curMessageID uint8
}

func newPublisherSubscriber(
	pub *Publisher,
	callerID string,
	tcpConn *prototcp.Conn,
	udpAddr *net.UDPAddr,
) *publisherSubscriber {
	ctx, ctxCancel := context.WithCancel(pub.ctx)

	ps := &publisherSubscriber{
		pub:       pub,
		callerID:  callerID,
		tcpConn:   tcpConn,
		udpAddr:   udpAddr,
		ctx:       ctx,
		ctxCancel: ctxCancel,
	}

	pub.subscribersWg.Add(1)
	go ps.run()

	return ps
}

func (ps *publisherSubscriber) run() {
	defer ps.pub.subscribersWg.Done()

	if ps.pub.conf.onSubscriber != nil {
		ps.pub.conf.onSubscriber()
	}

	if ps.tcpConn != nil {
		ps.runTCP()
	} else {
		ps.runUDP()
	}

	ps.ctxCancel()

	select {
	case ps.pub.subscriberClose <- ps:
	case <-ps.pub.ctx.Done():
	}
}

func (ps *publisherSubscriber) runTCP() {
	readerDone := make(chan struct{})
	go func() {
		defer close(readerDone)

		buf := make([]byte, 64)
		for {
			_, err := ps.tcpConn.NetConn().Read(buf)
			if err != nil {
				return
			}
		}
	}()

	select {
	case <-readerDone:
		ps.tcpConn.Close()

	case <-ps.ctx.Done():
		ps.tcpConn.Close()
		<-readerDone
	}
}

func (ps *publisherSubscriber) runUDP() {
	<-ps.ctx.Done()
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	if ps.tcpConn != nil {
		err := ps.tcpConn.WriteMessage(msg)
		if err != nil {
			ps.pub.conf.Node.Log(LogLevelError,
				"publisher '%s' is unable to write a TCP message to client '%s': %s",
				ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
				ps.tcpConn.NetConn().RemoteAddr(),
				err)
		}
	} else {
		ps.curMessageID++

		var rawMessage bytes.Buffer
		err := protocommon.MessageEncode(&rawMessage, msg)
		if err != nil {
			return
		}
		byts := rawMessage.Bytes()

		frames := protoudp.FramesForPayload(
			uint32(ps.pub.id),
			ps.curMessageID,
			byts)

		for _, f := range frames {
			err := ps.pub.conf.Node.udprosServer.WriteFrame(f, ps.udpAddr)
			if err != nil {
				ps.pub.conf.Node.Log(LogLevelError,
					"publisher '%s' is unable to write a UDP frame to client '%s': %s",
					ps.pub.conf.Node.absoluteTopicName(ps.pub.conf.Topic),
					ps.udpAddr,
					err)
			}
		}
	}
}

func (ps *publisherSubscriber) busInfo() []interface{} {
	proto := func() string {
		if ps.tcpConn != nil {
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
