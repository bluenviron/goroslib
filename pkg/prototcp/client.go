package prototcp

import (
	"context"
	"net"
	"time"
)

const (
	dialTimeout = 10 * time.Second
)

// NewClient connects to a TCPROS server and returns a Conn.
func NewClient(address string) (*Conn, error) {
	return NewClientContext(context.Background(), address)
}

// NewClientContext connects to a TCPROS server and returns a Conn.
// It allows to set a context that can be used to terminate the function.
func NewClientContext(ctx context.Context, address string) (*Conn, error) {
	d := net.Dialer{
		Timeout: dialTimeout,
	}
	nconn, err := d.DialContext(ctx, "tcp", address)
	if err != nil {
		return nil, err
	}

	return newConn(nconn), nil
}
