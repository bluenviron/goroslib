package prototcp

import (
	"net"
	"strconv"
	"time"
)

const (
	dialTimeout = 10 * time.Second
)

// NewClient connects to a TCPROS server and returns a Conn.
func NewClient(host string, port int) (*Conn, error) {
	nconn, err := net.DialTimeout("tcp4", host+":"+strconv.FormatInt(int64(port), 10), dialTimeout)
	if err != nil {
		return nil, err
	}

	return NewConn(nconn.(*net.TCPConn)), nil
}
