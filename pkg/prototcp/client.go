package prototcp

import (
	"net"
	"time"
)

const (
	dialTimeout = 10 * time.Second
)

// NewClient connects to a TCPROS server and returns a Conn.
func NewClient(address string) (*Conn, error) {
	nconn, err := net.DialTimeout("tcp", address, dialTimeout)
	if err != nil {
		return nil, err
	}

	return NewConn(nconn.(*net.TCPConn)), nil
}
