package prototcp

import (
	"io"
	"net"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/protocommon"
)

func TestConn(t *testing.T) {
	l, err := net.Listen("tcp", "localhost:9907")
	require.NoError(t, err)
	defer l.Close()

	serverDone := make(chan struct{})
	defer func() { <-serverDone }()

	go func() {
		defer close(serverDone)

		conn, err := l.Accept()
		require.NoError(t, err)
		defer conn.Close()

		raw, err := protocommon.HeaderRawDecode(conn)
		require.NoError(t, err)
		require.Equal(t, protocommon.HeaderRaw{
			"callerid": "mycallerid",
			"latching": "0",
			"md5sum":   "mysum",
			"topic":    "mytopic",
			"type":     "mytype",
		}, raw)

		err = protocommon.HeaderEncode(conn, &HeaderSubscriber{
			Callerid:          "mycallerid",
			Topic:             "mytopic",
			Type:              "mytype",
			Md5sum:            "mysum",
			MessageDefinition: "mydef",
			TcpNodelay:        1,
		})
		require.NoError(t, err)

		byt := make([]byte, 1)
		_, err = io.ReadFull(conn, byt)
		require.NoError(t, err)
		require.Equal(t, uint8(1), byt[0])

		var msg struct{}
		err = protocommon.MessageDecode(conn, &msg)
		require.NoError(t, err)
		require.Equal(t, struct{}{}, msg)

		_, err = conn.Write([]byte{1})
		require.NoError(t, err)

		err = protocommon.MessageEncode(conn, &struct{}{})
		require.NoError(t, err)
	}()

	conn, err := net.Dial("tcp", "localhost:9907")
	require.NoError(t, err)

	tconn := newConn(conn)
	defer tconn.Close()

	err = tconn.WriteHeader(&HeaderPublisher{
		Topic:    "mytopic",
		Type:     "mytype",
		Md5sum:   "mysum",
		Callerid: "mycallerid",
		Latching: 0,
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

	err = tconn.WriteServiceResState(1)
	require.NoError(t, err)

	err = tconn.WriteMessage(&struct{}{})
	require.NoError(t, err)

	state, err := tconn.ReadServiceResState()
	require.NoError(t, err)
	require.Equal(t, uint8(1), state)

	var msg struct{}
	err = tconn.ReadMessage(&msg)
	require.NoError(t, err)
	require.Equal(t, struct{}{}, msg)
}
