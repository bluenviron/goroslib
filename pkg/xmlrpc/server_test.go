package xmlrpc

import (
	"bytes"
	"net/http"
	"testing"

	"github.com/stretchr/testify/require"
)

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
