package goroslib

import (
	"bytes"
	"net"

	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
	"github.com/aler9/goroslib/pkg/protoudp"
)

type publisherSubscriber struct {
	pub          *Publisher
	callerId     string
	tcpClient    *prototcp.Conn
	udpAddr      *net.UDPAddr
	curMessageId uint8

	// in
	terminate chan struct{}
}

func newPublisherSubscriber(pub *Publisher, callerId string,
	tcpClient *prototcp.Conn, udpAddr *net.UDPAddr) {

	ps := &publisherSubscriber{
		pub:       pub,
		callerId:  callerId,
		tcpClient: tcpClient,
		udpAddr:   udpAddr,
		terminate: make(chan struct{}),
	}

	pub.subscribers[callerId] = ps

	pub.subscribersWg.Add(1)
	go ps.run()
}

func (ps *publisherSubscriber) close() {
	delete(ps.pub.subscribers, ps.callerId)
	close(ps.terminate)
}

func (ps *publisherSubscriber) run() {
	defer ps.pub.subscribersWg.Done()

	if ps.tcpClient != nil {
		ps.runTcp()
	} else {
		ps.runUdp()
	}
}

func (ps *publisherSubscriber) runTcp() {
	readerDone := make(chan struct{})
	go func() {
		defer close(readerDone)

		for {
			_, err := ps.tcpClient.ReadHeaderRaw()
			if err != nil {
				return
			}
		}
	}()

	select {
	case <-readerDone:
		ps.tcpClient.Close()
		ps.pub.subscriberTcpClose <- ps
		<-ps.terminate

	case <-ps.terminate:
		ps.tcpClient.Close()
		<-readerDone
	}
}

func (ps *publisherSubscriber) runUdp() {
	<-ps.terminate
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	if ps.tcpClient != nil {
		ps.tcpClient.WriteMessage(msg)

	} else {
		ps.curMessageId += 1

		rawMessage := bytes.NewBuffer(nil)
		err := protocommon.MessageEncode(rawMessage, msg)
		if err != nil {
			return
		}
		byts := rawMessage.Bytes()
		lbyts := len(byts)

		for i := 0; i < lbyts; i += protoudp.MAX_CONTENT_LENGTH {
			ps.pub.conf.Node.udprosServer.WriteFrame(&protoudp.Frame{
				ConnectionId: uint32(ps.pub.id),
				Opcode: func() protoudp.Opcode {
					if i == 0 {
						return protoudp.Data0
					}

					return protoudp.DataN
				}(),
				MessageId: ps.curMessageId,
				BlockId: func() uint16 {
					// return block count
					if i == 0 {
						if (lbyts % protoudp.MAX_CONTENT_LENGTH) == 0 {
							return uint16(lbyts / protoudp.MAX_CONTENT_LENGTH)
						}
						return uint16((lbyts / protoudp.MAX_CONTENT_LENGTH) + 1)
					}

					// return current block id
					return uint16(i / protoudp.MAX_CONTENT_LENGTH)
				}(),
				Content: func() []byte {
					j := i + protoudp.MAX_CONTENT_LENGTH
					if j > lbyts {
						j = lbyts
					}

					return byts[i:j]
				}(),
			}, ps.udpAddr)
		}
	}
}
