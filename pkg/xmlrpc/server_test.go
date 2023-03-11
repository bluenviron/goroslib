package xmlrpc

import (
	"bytes"
	"net"
	"net/http"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestServer(t *testing.T) {
	type myRequest struct {
		Param string
	}

	type myResponse struct {
		Param string
	}

	s, err := NewServer("localhost:9904", 5*time.Second)
	require.NoError(t, err)
	defer s.Close()

	require.Equal(t, "http://192.168.2.1:9904", s.URL(net.ParseIP("192.168.2.1"), ""))

	go s.Serve(func(raw *RequestRaw) interface{} {
		if raw.Method == "mymethod" {
			var req myRequest
			err := raw.Decode(&req)
			require.NoError(t, err)
			return myResponse{Param: "myresponse"}
		}
		return ErrorRes{}
	})

	t.Run("standard request", func(t *testing.T) {
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
	})

	t.Run("wrong path", func(t *testing.T) {
		res, err := http.Post("http://localhost:9904/wrong", "text/xml", nil)
		require.NoError(t, err)
		defer res.Body.Close()
		require.Equal(t, 404, res.StatusCode)
	})

	t.Run("wrong method", func(t *testing.T) {
		res, err := http.Get("http://localhost:9904/RPC2")
		require.NoError(t, err)
		defer res.Body.Close()
		require.Equal(t, 404, res.StatusCode)
	})

	t.Run("invalid request", func(t *testing.T) {
		var buf bytes.Buffer
		buf.WriteString("wrong")
		res, err := http.Post("http://localhost:9904/RPC2", "text/xml", &buf)
		require.NoError(t, err)
		defer res.Body.Close()
		require.Equal(t, 400, res.StatusCode)
	})

	t.Run("wrong xml method", func(t *testing.T) {
		var buf bytes.Buffer
		err = requestEncode(&buf, "wrong", myRequest{Param: "myrequest"})
		require.NoError(t, err)

		res, err := http.Post("http://localhost:9904/RPC2", "text/xml", &buf)
		require.NoError(t, err)
		defer res.Body.Close()
		require.Equal(t, 400, res.StatusCode)
	})
}

func TestServerServeAfterClose(t *testing.T) {
	s, err := NewServer("localhost:9904", 5*time.Second)
	require.NoError(t, err)
	s.Close()
	s.Serve(nil)
}

func TestServerError(t *testing.T) {
	s, err := NewServer("localhost:9904", 5*time.Second)
	require.NoError(t, err)
	defer s.Close()

	_, err = NewServer("localhost:9904", 5*time.Second)
	require.Error(t, err)
}
