package tcpros

import (
	"fmt"
	"net"
	"time"
)

const (
	_DIAL_TIMEOUT = 10 * time.Second
)

func NewClient(host string, port uint16) (*Conn, error) {
	nconn, err := net.DialTimeout("tcp", fmt.Sprintf("%s:%d", host, port), _DIAL_TIMEOUT)
	if err != nil {
		return nil, err
	}

	return NewConn(nconn), nil
}
