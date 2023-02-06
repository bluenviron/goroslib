package prototcp

import (
	"bytes"
	"io"
	"net"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/protocommon"
)

func TestConn(t *testing.T) {
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

	err = protocommon.HeaderEncode(&buf, &HeaderSubscriber{
		Callerid:          "mycallerid",
		Topic:             "mytopic",
		Type:              "mytype",
		Md5sum:            "mysum",
		MessageDefinition: "mydef",
		TcpNodelay:        1,
	})
	require.NoError(t, err)

	raw, err = tconn.ReadHeaderRaw()
	require.NoError(t, err)
	require.Equal(t, protocommon.HeaderRaw{
		"callerid":           "mycallerid",
		"md5sum":             "mysum",
		"message_definition": "mydef",
		"tcp_nodelay":        "1",
		"topic":              "mytopic",
		"type":               "mytype",
	}, raw)

	err = tconn.WriteServiceResponse(true, &struct{}{})
	require.NoError(t, err)

	byt := make([]byte, 1)
	_, err = io.ReadFull(&buf, byt)
	require.NoError(t, err)
	require.Equal(t, uint8(1), byt[0])

	var msg struct{}
	err = protocommon.MessageDecode(&buf, &msg)
	require.NoError(t, err)
	require.Equal(t, struct{}{}, msg)

	_, err = buf.Write([]byte{1})
	require.NoError(t, err)

	err = protocommon.MessageEncode(&buf, &struct{}{})
	require.NoError(t, err)

	var msg2 struct{}
	state, err := tconn.ReadServiceResponse(&msg2)
	require.NoError(t, err)
	require.Equal(t, true, state)
	require.Equal(t, struct{}{}, msg)

	// provider return with false state
	err = tconn.WriteServiceResponse(false, nil)
	require.NoError(t, err)

	// provider return a false state
	_, err = io.ReadFull(&buf, byt)
	require.NoError(t, err)
	require.Equal(t, uint8(0), byt[0])
}

func TestConnErrors(t *testing.T) {
	for _, ca := range []string{
		"invalid_header",
		"write_header",
		"read_service_response",
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
			tconn := NewConn(conn)
			conn.Close()

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

			case "read_service_response":
				var msg struct{}
				_, err = tconn.ReadServiceResponse(&msg)
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
