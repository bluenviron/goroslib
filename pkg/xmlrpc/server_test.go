package xmlrpc

import (
	"bytes"
	"net/http"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestServer(t *testing.T) {
	s, err := NewServer("127.0.0.1:8080")
	require.NoError(t, err)
	defer s.Close()

	type myRequest struct {
		Param string
	}

	type myResponse struct {
		Param string
	}

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

	res, err := http.Post("http://127.0.0.1:8080/RPC2", "text/xml", &buf)
	require.NoError(t, err)
	defer res.Body.Close()

	var xres myResponse
	err = responseDecode(res.Body, &xres)
	require.NoError(t, err)
	require.Equal(t, myResponse{Param: "myresponse"}, xres)
}
