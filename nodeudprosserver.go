package goroslib

import (
	"sync"
)

func (n *Node) runUdprosServer(wg *sync.WaitGroup) {
	defer wg.Done()

	for {
		frame, source, err := n.udprosServer.ReadFrame()
		if err != nil {
			break
		}

		n.udpFrame <- udpFrameReq{frame, source}
	}
}
