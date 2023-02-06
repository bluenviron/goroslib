package protoudp

import (
	"net"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestConn(t *testing.T) {
	pc, err := net.ListenPacket("udp", "127.0.0.1:9902") // localhost doesn't work with GitHub actions
	require.NoError(t, err)
	defer pc.Close()
	s := NewConn(pc)

	serverDone := make(chan struct{})
	defer func() { <-serverDone }()

	go func() {
		defer close(serverDone)

		fr, addr, err := s.ReadFrame()
		require.NoError(t, err)
		require.Equal(t, &Frame{
			ConnectionID: 3,
			MessageID:    2,
			BlockID:      1,
			Payload:      []byte{0x01, 0x02, 0x03, 0x04},
		}, fr)

		err = s.WriteMessage(5, 3, &struct{ A string }{"asd"}, addr)
		require.NoError(t, err)
	}()

	conn, err := net.Dial("udp", "127.0.0.1:9902")
	require.NoError(t, err)
	defer conn.Close()

	frames := framesForPayload(3, 2, []byte{0x01, 0x02, 0x03, 0x04})
	for _, f := range frames {
		_, err = conn.Write(f.encode())
		require.NoError(t, err)
	}

	buf := make([]byte, 1024)
	n, err := conn.Read(buf)
	require.NoError(t, err)

	var fr Frame
	err = fr.decode(buf[:n])
	require.NoError(t, err)
	require.Equal(t, Frame{
		ConnectionID: 5,
		MessageID:    3,
		BlockID:      1,
		Payload:      []byte{0x7, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0x61, 0x73, 0x64},
	}, fr)
}

func TestServerError(t *testing.T) {
	t.Run("read error", func(t *testing.T) {
		pc, err := net.ListenPacket("udp", "127.0.0.1:9902")
		require.NoError(t, err)
		s := NewConn(pc)

		done := make(chan struct{})
		go func() {
			defer close(done)
			_, _, err := s.ReadFrame()
			require.Error(t, err)
		}()

		pc.Close()
		<-done
	})

	t.Run("invalid frame", func(t *testing.T) {
		pc, err := net.ListenPacket("udp", "127.0.0.1:9902")
		require.NoError(t, err)
		defer pc.Close()
		s := NewConn(pc)

		done := make(chan struct{})
		go func() {
			defer close(done)
			_, _, err := s.ReadFrame()
			require.Error(t, err)
		}()

		conn, err := net.Dial("udp", "127.0.0.1:9902")
		require.NoError(t, err)

		_, err = conn.Write([]byte{0x01})
		require.NoError(t, err)

		<-done
	})
}
