package goroslib

import (
	"sync"

	"github.com/aler9/goroslib/protocommon"
	"github.com/aler9/goroslib/prototcp"
)

func (n *Node) runTcprosClient(wg *sync.WaitGroup, client *prototcp.Conn) {
	ok := func() bool {
		rawHeader, err := client.ReadHeaderRaw()
		if err != nil {
			return false
		}

		if _, ok := rawHeader["topic"]; ok {
			var header prototcp.HeaderSubscriber
			err = protocommon.HeaderDecode(rawHeader, &header)
			if err != nil {
				return false
			}

			n.tcpClientSubscriber <- tcpClientSubscriberReq{
				client: client,
				header: &header,
			}
			return true

		} else if _, ok := rawHeader["service"]; ok {
			var header prototcp.HeaderServiceClient
			err = protocommon.HeaderDecode(rawHeader, &header)
			if err != nil {
				return false
			}

			n.tcpClientServiceClient <- tcpClientServiceClientReq{
				client: client,
				header: &header,
			}
			return true
		}

		return false
	}()
	if !ok {
		client.Close()
		n.tcpClientClose <- client
	}

	wg.Done()
}
