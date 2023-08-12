package prototcp

import (
	"bytes"
	"fmt"
	"io"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/protocommon"
)

func TestConn(t *testing.T) {
	t.Run("write header", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		err := tconn.WriteHeader(&HeaderPublisher{
			Topic:             "mytopic",
			Type:              "mytype",
			Md5sum:            "mysum",
			Callerid:          "mycallerid",
			Latching:          0,
			MessageDefinition: "mydef",
		})
		require.NoError(t, err)

		raw, err := protocommon.HeaderRawDecode(&buf)
		require.NoError(t, err)
		require.Equal(t, protocommon.HeaderRaw{
			"callerid":           "mycallerid",
			"latching":           "0",
			"md5sum":             "mysum",
			"topic":              "mytopic",
			"type":               "mytype",
			"message_definition": "mydef",
		}, raw)
	})

	t.Run("read header", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		err := protocommon.HeaderEncode(&buf, &HeaderSubscriber{
			Callerid:          "mycallerid",
			Topic:             "mytopic",
			Type:              "mytype",
			Md5sum:            "mysum",
			MessageDefinition: "mydef",
			TcpNodelay:        1,
		})
		require.NoError(t, err)

		raw, err := tconn.ReadHeaderRaw()
		require.NoError(t, err)
		require.Equal(t, protocommon.HeaderRaw{
			"callerid":           "mycallerid",
			"md5sum":             "mysum",
			"message_definition": "mydef",
			"tcp_nodelay":        "1",
			"topic":              "mytopic",
			"type":               "mytype",
		}, raw)
	})

	t.Run("write message", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		err := tconn.WriteMessage(&struct{}{})
		require.NoError(t, err)

		var msg struct{}
		err = protocommon.MessageDecode(&buf, &msg)
		require.NoError(t, err)
		require.Equal(t, struct{}{}, msg)
	})

	t.Run("read message", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		err := protocommon.MessageEncode(&buf, &struct{}{})
		require.NoError(t, err)

		var msg2 struct{}
		err = tconn.ReadMessage(&msg2)
		require.NoError(t, err)
		require.Equal(t, struct{}{}, msg2)
	})

	t.Run("write service response with state = true", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		err := tconn.WriteServiceResponse(true, &struct{}{})
		require.NoError(t, err)

		byt := make([]byte, 1)
		_, err = io.ReadFull(&buf, byt)
		require.NoError(t, err)
		require.Equal(t, uint8(1), byt[0])

		var msg struct{}
		err = protocommon.MessageDecode(&buf, &msg)
		require.NoError(t, err)
		require.Equal(t, struct{}{}, msg)
	})

	t.Run("read service response with state = true", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		_, err := buf.Write([]byte{1})
		require.NoError(t, err)

		err = protocommon.MessageEncode(&buf, &struct{}{})
		require.NoError(t, err)

		var msg2 struct{}
		state, err := tconn.ReadServiceResponse(&msg2)
		require.NoError(t, err)
		require.Equal(t, true, state)
		require.Equal(t, struct{}{}, msg2)
	})

	t.Run("write service response with state = false", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		err := tconn.WriteServiceResponse(false, nil)
		require.NoError(t, err)

		byt := make([]byte, 1)
		_, err = io.ReadFull(&buf, byt)
		require.NoError(t, err)
		require.Equal(t, uint8(0), byt[0])
	})

	t.Run("read service response with state = false", func(t *testing.T) {
		var buf bytes.Buffer
		tconn := NewConn(&buf)

		_, err := buf.Write([]byte{0})
		require.NoError(t, err)

		var msg2 struct{}
		state, err := tconn.ReadServiceResponse(&msg2)
		require.NoError(t, err)
		require.Equal(t, false, state)
		require.Equal(t, struct{}{}, msg2)
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

func TestConnErrors(t *testing.T) {
	t.Run("header write invalid", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 0})
		err := tconn.WriteHeader(123)
		require.Error(t, err)
	})

	t.Run("header write error", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 0})
		err := tconn.WriteHeader(&HeaderPublisher{
			Topic:             "mytopic",
			Type:              "mytype",
			Md5sum:            "mysum",
			Callerid:          "mycallerid",
			Latching:          0,
			MessageDefinition: "mydef",
		})
		require.Error(t, err)
	})

	t.Run("service response write error 1", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 0})
		err := tconn.WriteServiceResponse(true, &struct{}{})
		require.Error(t, err)
	})

	t.Run("service response write error 2", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 2})
		err := tconn.WriteServiceResponse(true, &struct{}{})
		require.Error(t, err)
	})

	t.Run("service response read error", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 0})
		var msg struct{}
		_, err := tconn.ReadServiceResponse(&msg)
		require.Error(t, err)
	})

	t.Run("message write invalid", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 0})
		err := tconn.WriteMessage(123)
		require.Error(t, err)
	})

	t.Run("message write error", func(t *testing.T) {
		tconn := NewConn(&limitedReadWriter{cap: 0})
		err := tconn.WriteMessage(&struct{}{})
		require.Error(t, err)
	})
}
