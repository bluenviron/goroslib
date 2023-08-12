package protoudp

import (
	"bytes"
	"fmt"
	"io"
	"net"
	"testing"

	"github.com/stretchr/testify/require"
)

type testPacketConn struct {
	buf io.ReadWriter
}

func (c *testPacketConn) ReadFrom(p []byte) (int, net.Addr, error) {
	n, err := c.buf.Read(p)
	return n, &net.UDPAddr{}, err
}

func (c *testPacketConn) WriteTo(p []byte, _ net.Addr) (int, error) {
	return c.buf.Write(p)
}

func TestConn(t *testing.T) {
	t.Run("read", func(t *testing.T) {
		var buf bytes.Buffer
		conn := NewConn(&testPacketConn{buf: &buf})

		frames := framesForPayload(3, 2, []byte{0x01, 0x02, 0x03, 0x04})
		for _, f := range frames {
			_, err := buf.Write(f.encode())
			require.NoError(t, err)
		}

		fr, _, err := conn.ReadFrame()
		require.NoError(t, err)
		require.Equal(t, &Frame{
			ConnectionID: 3,
			MessageID:    2,
			BlockID:      1,
			Payload:      []byte{0x01, 0x02, 0x03, 0x04},
		}, fr)
	})

	t.Run("write", func(t *testing.T) {
		var buf bytes.Buffer
		conn := NewConn(&testPacketConn{buf: &buf})

		err := conn.WriteMessage(5, 3, &struct{ A string }{"asd"}, nil)
		require.NoError(t, err)

		buf2 := make([]byte, 1024)
		n, err := buf.Read(buf2)
		require.NoError(t, err)

		var fr Frame
		err = fr.decode(buf2[:n])
		require.NoError(t, err)
		require.Equal(t, Frame{
			ConnectionID: 5,
			MessageID:    3,
			BlockID:      1,
			Payload:      []byte{0x7, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0x61, 0x73, 0x64},
		}, fr)
	})
}

type limitedReadWriter struct {
	cap int
	n   int
}

func (b *limitedReadWriter) Read(_ []byte) (int, error) {
	return 0, fmt.Errorf("unimplemented")
}

func (b *limitedReadWriter) Write(p []byte) (int, error) {
	b.n += len(p)
	if b.n > b.cap {
		return 0, fmt.Errorf("capacity reached")
	}
	return len(p), nil
}

func TestConnError(t *testing.T) {
	t.Run("frame read error", func(t *testing.T) {
		conn := NewConn(&testPacketConn{buf: &limitedReadWriter{cap: 0}})
		_, _, err := conn.ReadFrame()
		require.Error(t, err)
	})

	t.Run("frame read invalid", func(t *testing.T) {
		var buf bytes.Buffer
		buf.Write([]byte{1})

		conn := NewConn(&testPacketConn{buf: &buf})
		_, _, err := conn.ReadFrame()
		require.Error(t, err)
	})

	t.Run("frame write invalid", func(t *testing.T) {
		var buf bytes.Buffer

		conn := NewConn(&testPacketConn{buf: &buf})
		err := conn.WriteMessage(1, 2, 123, nil)
		require.Error(t, err)
	})

	t.Run("frame write error", func(t *testing.T) {
		conn := NewConn(&testPacketConn{buf: &limitedReadWriter{cap: 0}})
		err := conn.WriteMessage(1, 2, &struct{}{}, nil)
		require.Error(t, err)
	})
}
