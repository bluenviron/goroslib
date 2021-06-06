package prototcp

import (
	"net"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestServer(t *testing.T) {
	s, err := NewServer("localhost:9901")
	require.NoError(t, err)
	defer s.Close()

	serverDone := make(chan struct{})
	defer func() { <-serverDone }()

	go func() {
		defer close(serverDone)

		conn, err := s.Accept()
		require.NoError(t, err)
		defer conn.Close()
	}()

	conn, err := net.Dial("tcp", "localhost:9901")
	require.NoError(t, err)
	defer conn.Close()
}
