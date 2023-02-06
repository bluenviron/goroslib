package goroslib

import (
	"sync"
)

func (n *Node) runUdprosServer(wg *sync.WaitGroup) {
	defer wg.Done()

	for {
		frame, source, err := n.udprosConn.ReadFrame()
		if err != nil {
			break
		}

		select {
		case n.udpFrame <- udpFrameReq{frame, source}:
		case <-n.ctx.Done():
		}
	}
}
