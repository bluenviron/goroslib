package xmlrpc

import (
	"bytes"
	"net"
	"net/http"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestServerURL(t *testing.T) {
	u := ServerURL(
		net.ParseIP("192.168.2.1"),
		123,
		"")
	require.Equal(t, "http://192.168.2.1:123", u)
}

func TestServer(t *testing.T) {
	type myRequest struct {
		Param string
	}

	type myResponse struct {
		Param string
	}

	s, err := NewServer("localhost:9904")
	require.NoError(t, err)
	defer s.Close()

	require.NotEqual(t, 0, s.Port())

	go s.Serve(func(raw *RequestRaw) interface{} {
		require.Equal(t, "mymethod", raw.Method)

		var req myRequest
		err := raw.Decode(&req)
		require.NoError(t, err)

		return myResponse{Param: "myresponse"}
	})

	var buf bytes.Buffer
	err = requestEncode(&buf, "mymethod", myRequest{Param: "myrequest"})
	require.NoError(t, err)

	res, err := http.Post("http://localhost:9904/RPC2", "text/xml", &buf)
	require.NoError(t, err)
	defer res.Body.Close()

	var xres myResponse
	err = responseDecode(res.Body, &xres)
	require.NoError(t, err)
	require.Equal(t, myResponse{Param: "myresponse"}, xres)
}

func TestServerError(t *testing.T) {
	s, err := NewServer("localhost:9904")
	require.NoError(t, err)
	defer s.Close()

	_, err = NewServer("localhost:9904")
	require.Error(t, err)
}
