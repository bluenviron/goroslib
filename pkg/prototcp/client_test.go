package prototcp

import (
	"net"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestClient(t *testing.T) {
	l, err := net.Listen("tcp", "127.0.0.1:9900")
	require.NoError(t, err)
	defer l.Close()

	serverDone := make(chan struct{})
	defer func() { <-serverDone }()

	go func() {
		defer close(serverDone)

		conn, err := l.Accept()
		require.NoError(t, err)
		defer conn.Close()
	}()

	c, err := NewClient("127.0.0.1:9900")
	require.NoError(t, err)
	defer c.Close()
}
