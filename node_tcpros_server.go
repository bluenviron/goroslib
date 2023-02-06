package goroslib

import (
	"net"
	"sync"
	"time"

	"github.com/aler9/goroslib/pkg/protocommon"
	"github.com/aler9/goroslib/pkg/prototcp"
)

func (n *Node) runTcprosServer(wg *sync.WaitGroup) {
	defer wg.Done()

	for {
		conn, err := n.tcprosListener.Accept()
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

func (n *Node) runTcprosServerConn(wg *sync.WaitGroup, nconn net.Conn) {
	defer wg.Done()

	ok := func() bool {
		tconn := prototcp.NewConn(nconn)

		nconn.SetReadDeadline(time.Now().Add(readTimeout))
		rawHeader, err := tconn.ReadHeaderRaw()
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
				nconn:  nconn,
				tconn:  tconn,
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
				nconn:  nconn,
				tconn:  tconn,
				header: &header,
			}:
			case <-n.ctx.Done():
			}
			return true
		}

		return false
	}()
	if !ok {
		nconn.Close()

		select {
		case n.tcpConnClose <- nconn:
		case <-n.ctx.Done():
		}
	}
}
