package prototcp

import (
	"net"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestServerURL(t *testing.T) {
	u := ServerURL(
		net.ParseIP("192.168.2.1"),
		123,
		"")
	require.Equal(t, "rosrpc://192.168.2.1:123", u)
}

func TestServer(t *testing.T) {
	serverDone := make(chan struct{})
	defer func() { <-serverDone }()

	s, err := NewServer("localhost:9901")
	require.NoError(t, err)
	defer s.Close()

	require.NotEqual(t, 0, s.Port())

	go func() {
		defer close(serverDone)

		for {
			conn, err := s.Accept()
			if err != nil {
				return
			}
			conn.Close()
		}
	}()

	conn, err := net.Dial("tcp", "localhost:9901")
	require.NoError(t, err)
	defer conn.Close()
}

func TestServerError(t *testing.T) {
	s1, err := NewServer("localhost:9901")
	require.NoError(t, err)
	defer s1.Close()

	_, err = NewServer("localhost:9901")
	require.Error(t, err)
}
