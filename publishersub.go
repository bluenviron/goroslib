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
	callerID     string
	tcpClient    *prototcp.Conn
	udpAddr      *net.UDPAddr
	curMessageID uint8

	// in
	terminate chan struct{}
}

func newPublisherSubscriber(pub *Publisher, callerID string,
	tcpClient *prototcp.Conn, udpAddr *net.UDPAddr) {

	ps := &publisherSubscriber{
		pub:       pub,
		callerID:  callerID,
		tcpClient: tcpClient,
		udpAddr:   udpAddr,
		terminate: make(chan struct{}),
	}

	pub.subscribers[callerID] = ps

	pub.subscribersWg.Add(1)
	go ps.run()
}

func (ps *publisherSubscriber) close() {
	delete(ps.pub.subscribers, ps.callerID)
	close(ps.terminate)
}

func (ps *publisherSubscriber) run() {
	defer ps.pub.subscribersWg.Done()

	if ps.tcpClient != nil {
		ps.runTCP()
	} else {
		ps.runUDP()
	}
}

func (ps *publisherSubscriber) runTCP() {
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
		ps.pub.subscriberTCPClose <- ps
		<-ps.terminate

	case <-ps.terminate:
		ps.tcpClient.Close()
		<-readerDone
	}
}

func (ps *publisherSubscriber) runUDP() {
	<-ps.terminate
}

func (ps *publisherSubscriber) writeMessage(msg interface{}) {
	if ps.tcpClient != nil {
		ps.tcpClient.WriteMessage(msg)

	} else {
		ps.curMessageID++

		rawMessage := bytes.NewBuffer(nil)
		err := protocommon.MessageEncode(rawMessage, msg)
		if err != nil {
			return
		}
		byts := rawMessage.Bytes()
		lbyts := len(byts)

		for i := 0; i < lbyts; i += protoudp.MaxPayloadSize {
			ps.pub.conf.Node.udprosServer.WriteFrame(&protoudp.Frame{
				ConnectionID: uint32(ps.pub.id),
				Opcode: func() protoudp.Opcode {
					if i == 0 {
						return protoudp.Data0
					}

					return protoudp.DataN
				}(),
				MessageID: ps.curMessageID,
				BlockID: func() uint16 {
					// return block count
					if i == 0 {
						if (lbyts % protoudp.MaxPayloadSize) == 0 {
							return uint16(lbyts / protoudp.MaxPayloadSize)
						}
						return uint16((lbyts / protoudp.MaxPayloadSize) + 1)
					}

					// return current block id
					return uint16(i / protoudp.MaxPayloadSize)
				}(),
				Content: func() []byte {
					j := i + protoudp.MaxPayloadSize
					if j > lbyts {
						j = lbyts
					}

					return byts[i:j]
				}(),
			}, ps.udpAddr)
		}
	}
}
