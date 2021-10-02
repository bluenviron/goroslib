package prototcp

import (
	"net"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestServer(t *testing.T) {
	serverDone := make(chan struct{})
	defer func() { <-serverDone }()

	s, err := NewServer("localhost:9901", net.ParseIP("192.168.2.1"), "")
	require.NoError(t, err)
	defer s.Close()

	require.Equal(t, "rosrpc://192.168.2.1:9901", s.URL())

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
	s1, err := NewServer("localhost:9901", net.ParseIP("127.0.0.1"), "")
	require.NoError(t, err)
	defer s1.Close()

	_, err = NewServer("localhost:9901", net.ParseIP("127.0.0.1"), "")
	require.Error(t, err)
}
