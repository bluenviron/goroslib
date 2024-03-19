package xmlrpc

import (
	"context"
	"net"
	"net/http"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestClient(t *testing.T) {
	// since the HTTP server is created and deleted multiple times,
	// we can't reuse TCP connections.
	http.DefaultTransport.(*http.Transport).DisableKeepAlives = true

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
			err = responseEncode(w, res)
			require.NoError(t, err)
		}),
	}
	defer hs.Shutdown(context.Background())

	l, err := net.Listen("tcp", "localhost:9903")
	require.NoError(t, err)

	go hs.Serve(l)

	c := NewClient("localhost:9903", &http.Client{})
	var res myResponse
	err = c.Do("mymethod", myRequest{Param: "myparam"}, &res)
	require.NoError(t, err)
	require.Equal(t, myResponse{Param: "myparam"}, res)
}

func TestClientErrors(t *testing.T) {
	t.Run("invalid request", func(t *testing.T) {
		c := NewClient("localhost:9903", &http.Client{})
		err := c.Do("mymethod", struct {
			Param int64
		}{123}, nil)
		require.EqualError(t, err, "unhandled value type: int64")
	})

	t.Run("server error", func(t *testing.T) {
		c := NewClient("127.0.0.1:9903", &http.Client{})
		err := c.Do("mymethod", struct {
			Param int
		}{123}, nil)
		require.EqualError(t, err, "Post \"http://127.0.0.1:9903/RPC2\": "+
			"dial tcp 127.0.0.1:9903: connect: connection refused")
	})

	t.Run("bad status code", func(t *testing.T) {
		type myResponse struct {
			Param string
		}

		l, err := net.Listen("tcp", "localhost:9903")
		require.NoError(t, err)
		defer l.Close()

		hs := &http.Server{
			Handler: http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
				w.WriteHeader(http.StatusBadRequest)
			}),
		}
		defer hs.Shutdown(context.Background())

		go hs.Serve(l)

		c := NewClient("localhost:9903", &http.Client{})
		err = c.Do("mymethod", struct {
			Param int
		}{123}, &myResponse{})
		require.EqualError(t, err, "bad status code: 400")
	})

	t.Run("invalid response", func(t *testing.T) {
		type myResponse struct {
			Param string
		}

		l, err := net.Listen("tcp", "localhost:9903")
		require.NoError(t, err)
		defer l.Close()

		hs := &http.Server{
			Handler: http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
				w.Write([]byte("invalid"))
			}),
		}
		defer hs.Shutdown(context.Background())

		go hs.Serve(l)

		c := NewClient("localhost:9903", &http.Client{})
		err = c.Do("mymethod", struct {
			Param int
		}{123}, &myResponse{})
		require.EqualError(t, err, "expected xml.ProcInst, got xml.CharData")
	})
}
