package xmlrpc

import (
	"context"
	"net"
	"net/http"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestClient(t *testing.T) {
	type myRequest struct {
		Param string
	}

	type myResponse struct {
		Param string
	}

	hs := &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
			require.Equal(t, "/RPC2", req.URL.Path)
			require.Equal(t, "POST", req.Method)

			raw, err := requestDecodeRaw(req.Body)
			require.NoError(t, err)
			require.Equal(t, "mymethod", raw.Method)

			var mreq myRequest
			err = raw.Decode(&mreq)
			require.NoError(t, err)
			require.Equal(t, "myparam", mreq.Param)

			res := myResponse{Param: "myparam"}
			responseEncode(w, res)
		}),
	}
	defer hs.Shutdown(context.Background())

	l, err := net.Listen("tcp", "localhost:9903")
	require.NoError(t, err)
	defer l.Close()

	go hs.Serve(l)

	c := NewClient("localhost:9903")
	var res myResponse
	err = c.Do("mymethod", myRequest{Param: "myparam"}, &res)
	require.NoError(t, err)
	require.Equal(t, myResponse{Param: "myparam"}, res)
}

func TestClientErrors(t *testing.T) {
	t.Run("invalid request", func(t *testing.T) {
		c := NewClient("localhost:9908")
		err := c.Do("mymethod", struct {
			Param int64
		}{123}, nil)
		require.Equal(t, "unhandled value type: int64", err.Error())
	})

	t.Run("server error", func(t *testing.T) {
		c := NewClient("127.0.0.1:9908")
		err := c.Do("mymethod", struct {
			Param int
		}{123}, nil)
		require.Equal(t, "Post \"http://127.0.0.1:9908/RPC2\": "+
			"dial tcp 127.0.0.1:9908: connect: connection refused", err.Error())
	})

	t.Run("invalid response", func(t *testing.T) {
		type myResponse struct {
			Param string
		}

		hs := &http.Server{
			Handler: http.HandlerFunc(func(w http.ResponseWriter, req *http.Request) {
				w.Write([]byte("invalid"))
			}),
		}
		defer hs.Shutdown(context.Background())

		l, err := net.Listen("tcp", "localhost:9908")
		require.NoError(t, err)
		defer l.Close()

		go hs.Serve(l)

		c := NewClient("localhost:9908")
		err = c.Do("mymethod", struct {
			Param int
		}{123}, &myResponse{})
		require.Equal(t, "expected xml.ProcInst, got xml.CharData", err.Error())
	})
}
