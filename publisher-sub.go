package goroslib

import (
	"bytes"
	"fmt"
	"net"

	"github.com/aler9/goroslib/proto-common"
	"github.com/aler9/goroslib/proto-tcp"
	"github.com/aler9/goroslib/proto-udp"
)

type publisherSubscriber struct {
	pub          *Publisher
	callerid     string
	tcpClient    *proto_tcp.Conn
	udpAddr      *net.UDPAddr
	curMessageId uint8

	terminate chan struct{}
	done      chan struct{}
}

func newPublisherSubscriber(pub *Publisher, callerid string, tcpClient *proto_tcp.Conn,
	udpHost string, udpPort int) *publisherSubscriber {

	var udpAddr *net.UDPAddr
	if tcpClient == nil {
		udpAddr, _ = net.ResolveUDPAddr("udp", fmt.Sprintf("%s:%d", udpHost, udpPort))
	}

	ps := &publisherSubscriber{
		pub:       pub,
		callerid:  callerid,
		tcpClient: tcpClient,
		udpAddr:   udpAddr,
		terminate: make(chan struct{}),
		done:      make(chan struct{}),
	}

	go ps.run()

	return ps
}

func (ps *publisherSubscriber) run() {
	if ps.tcpClient != nil {
	outer:
		for {
			_, err := ps.tcpClient.ReadHeaderRaw()
			if err != nil {
				break outer
			}
		}

		ps.tcpClient.Close()

		ps.pub.events <- publisherEventSubscriberTcpClose{ps}

	} else {
		<-ps.terminate
	}

	close(ps.done)
}

func (ps *publisherSubscriber) close() {
	if ps.tcpClient != nil {
		ps.tcpClient.Close()
	} else {
		close(ps.terminate)
	}
	<-ps.done
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	if ps.tcpClient != nil {
		ps.tcpClient.WriteMessage(msg)
	} else {
		ps.curMessageId += 1

		rawMessage := bytes.NewBuffer(nil)
		err := proto_common.MessageEncode(rawMessage, msg)
		if err != nil {
			return
		}

		f := &proto_udp.Frame{
			ConnectionId: uint32(ps.pub.id),
			Opcode:       0,
			MessageId:    ps.curMessageId,
			BlockId:      1,
			RawMessage:   rawMessage.Bytes(),
		}

		ps.pub.conf.Node.udprosServer.WriteFrame(f, ps.udpAddr)
	}
}
