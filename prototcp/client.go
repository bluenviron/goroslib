package prototcp

import (
	"net"
	"strconv"
	"time"
)

const (
	_DIAL_TIMEOUT = 10 * time.Second
)

func NewClient(host string, port int) (*Conn, error) {
	nconn, err := net.DialTimeout("tcp4", host+":"+strconv.FormatInt(int64(port), 10), _DIAL_TIMEOUT)
	if err != nil {
		return nil, err
	}

	return NewConn(nconn), nil
}
