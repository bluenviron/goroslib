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
			"callerid":           "mycallerid",
			"latching":           "0",
			"md5sum":             "mysum",
			"topic":              "mytopic",
			"type":               "mytype",
			"message_definition": "mydef",
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

	require.NotEqual(t, nil, tconn.NetConn())

	err = tconn.WriteHeader(&HeaderPublisher{
		Topic:             "mytopic",
		Type:              "mytype",
		Md5sum:            "mysum",
		Callerid:          "mycallerid",
		Latching:          0,
		MessageDefinition: "mydef",
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

	err = tconn.WriteServiceResState(true)
	require.NoError(t, err)

	err = tconn.WriteMessage(&struct{}{})
	require.NoError(t, err)

	state, err := tconn.ReadServiceResState()
	require.NoError(t, err)
	require.Equal(t, true, state)

	var msg struct{}
	err = tconn.ReadMessage(&msg, true)
	require.NoError(t, err)
	require.Equal(t, struct{}{}, msg)
}

func TestConnErrors(t *testing.T) {
	for _, ca := range []string{
		"invalid_header",
		"write_header",
		"read_service_res_state",
		"invalid_message",
		"write_message",
	} {
		t.Run(ca, func(t *testing.T) {
			l, err := net.Listen("tcp", "localhost:9907")
			require.NoError(t, err)
			defer l.Close()

			serverDone := make(chan struct{})
			defer func() { <-serverDone }()

			go func() {
				defer close(serverDone)

				conn, err := l.Accept()
				require.NoError(t, err)
				conn.Close()
			}()

			conn, err := net.Dial("tcp", "localhost:9907")
			require.NoError(t, err)

			tconn := newConn(conn)
			tconn.Close()

			switch ca {
			case "invalid_header":
				err := tconn.WriteHeader(123)
				require.Error(t, err)

			case "write_header":
				err := tconn.WriteHeader(&HeaderPublisher{
					Topic:             "mytopic",
					Type:              "mytype",
					Md5sum:            "mysum",
					Callerid:          "mycallerid",
					Latching:          0,
					MessageDefinition: "mydef",
				})
				require.Error(t, err)

			case "read_service_res_state":
				_, err = tconn.ReadServiceResState()
				require.Error(t, err)

			case "invalid_message":
				err := tconn.WriteMessage(123)
				require.Error(t, err)

			case "write_message":
				err := tconn.WriteMessage(&struct{}{})
				require.Error(t, err)
			}
		})
	}
}
