package goroslib

import (
	"sync"

	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
)

func (n *Node) runTcprosServer(wg *sync.WaitGroup) {
	defer wg.Done()

	for {
		conn, err := n.tcprosServer.Accept()
		if err != nil {
			break
		}

		select {
		case n.tcpConnNew <- conn:
		case <-n.ctx.Done():
			conn.Close()
		}
	}
}

func (n *Node) runTcprosServerConn(wg *sync.WaitGroup, conn *prototcp.Conn) {
	defer wg.Done()

	ok := func() bool {
		rawHeader, err := conn.ReadHeaderRaw()
		if err != nil {
			return false
		}

		if _, ok := rawHeader["topic"]; ok {
			var header prototcp.HeaderSubscriber
			err = protocommon.HeaderDecode(rawHeader, &header)
			if err != nil {
				return false
			}

			select {
			case n.tcpConnSubscriber <- tcpConnSubscriberReq{
				conn:   conn,
				header: &header,
			}:
			case <-n.ctx.Done():
			}
			return true

		} else if _, ok := rawHeader["service"]; ok {
			var header prototcp.HeaderServiceClient
			err = protocommon.HeaderDecode(rawHeader, &header)
			if err != nil {
				return false
			}

			select {
			case n.tcpConnServiceClient <- tcpConnServiceClientReq{
				conn:   conn,
				header: &header,
			}:
			case <-n.ctx.Done():
			}
			return true
		}

		return false
	}()
	if !ok {
		conn.Close()

		select {
		case n.tcpConnClose <- conn:
		case <-n.ctx.Done():
		}
	}
}
