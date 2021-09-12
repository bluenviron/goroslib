package apislave

import (
	"bytes"
	"net/http"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

func TestServer(t *testing.T) {
	s, err := NewServer("localhost:9906")
	require.NoError(t, err)
	defer s.Close()

	require.NotEqual(t, 0, s.Port())

	go s.Serve(func(req Request) Response {
		switch req.(type) {
		case *RequestGetBusInfo:
			require.Equal(t, &RequestGetBusInfo{CallerID: "mycaller"}, req)
			return ResponseGetBusInfo{Code: 1}

		case *RequestGetPid:
			require.Equal(t, &RequestGetPid{CallerID: "mycaller"}, req)
			return ResponseGetPid{Code: 1}

		case *RequestGetPublications:
			require.Equal(t, &RequestGetPublications{CallerID: "mycaller"}, req)
			return ResponseGetPublications{Code: 1}

		case *RequestPublisherUpdate:
			require.Equal(t, &RequestPublisherUpdate{CallerID: "mycaller"}, req)
			return ResponsePublisherUpdate{Code: 1}

		case *RequestRequestTopic:
			require.Equal(t, &RequestRequestTopic{CallerID: "mycaller"}, req)
			return ResponseRequestTopic{Code: 1}

		case *RequestShutdown:
			require.Equal(t, &RequestShutdown{CallerID: "mycaller"}, req)
			return ResponseShutdown{Code: 1}
		}

		return ErrorRes{}
	})

	c := xmlrpc.NewClient("localhost:9906")

	func() {
		var res ResponseGetBusInfo
		err = c.Do("getBusInfo", RequestGetBusInfo{CallerID: "mycaller"}, &res)
		require.NoError(t, err)
		require.Equal(t, ResponseGetBusInfo{Code: 1}, res)
	}()

	func() {
		var res ResponseGetPid
		err = c.Do("getPid", RequestGetPid{CallerID: "mycaller"}, &res)
		require.NoError(t, err)
		require.Equal(t, ResponseGetPid{Code: 1}, res)
	}()

	func() {
		var res ResponseGetPublications
		err = c.Do("getPublications", RequestGetPublications{CallerID: "mycaller"}, &res)
		require.NoError(t, err)
		require.Equal(t, ResponseGetPublications{Code: 1}, res)
	}()

	func() {
		var res ResponsePublisherUpdate
		err = c.Do("publisherUpdate", RequestPublisherUpdate{CallerID: "mycaller"}, &res)
		require.NoError(t, err)
		require.Equal(t, ResponsePublisherUpdate{Code: 1}, res)
	}()

	func() {
		var res ResponseRequestTopic
		err = c.Do("requestTopic", RequestRequestTopic{CallerID: "mycaller"}, &res)
		require.NoError(t, err)
		require.Equal(t, ResponseRequestTopic{Code: 1}, res)
	}()

	func() {
		var res ResponseShutdown
		err = c.Do("shutdown", RequestShutdown{CallerID: "mycaller"}, &res)
		require.NoError(t, err)
		require.Equal(t, ResponseShutdown{Code: 1}, res)
	}()
}

func TestServerErrors(t *testing.T) {
	t.Run("double listen", func(t *testing.T) {
		s, err := NewServer("localhost:9906")
		require.NoError(t, err)
		defer s.Close()

		_, err = NewServer("localhost:9906")
		require.Error(t, err)
	})

	t.Run("invalid method", func(t *testing.T) {
		s, err := NewServer("localhost:9906")
		require.NoError(t, err)
		defer s.Close()

		go s.Serve(func(req Request) Response {
			return ErrorRes{}
		})

		c := xmlrpc.NewClient("localhost:9906")

		var res ResponseGetBusInfo
		err = c.Do("invalidMethod", RequestGetBusInfo{CallerID: "mycaller"}, &res)
		require.Error(t, err)
	})

	t.Run("invalid payload 1", func(t *testing.T) {
		s, err := NewServer("localhost:9909")
		require.NoError(t, err)
		defer s.Close()

		go s.Serve(func(req Request) Response {
			return ErrorRes{}
		})

		var buf bytes.Buffer
		buf.Write([]byte(`<?xml version="1.0"?><methodCall><methodName>testMethodName</methodName><params>`))
		res, err := http.Post("http://localhost:9909/RPC2", "text/xml", &buf)
		require.NoError(t, err)
		defer res.Body.Close()
		require.Equal(t, 400, res.StatusCode)
	})

	t.Run("invalid payload 2", func(t *testing.T) {
		s, err := NewServer("localhost:9909")
		require.NoError(t, err)
		defer s.Close()

		go s.Serve(func(req Request) Response {
			if _, ok := req.(*RequestShutdown); ok {
				require.Equal(t, &RequestShutdown{CallerID: "mycaller"}, req)
				return ResponseShutdown{Code: 1}
			}

			return ErrorRes{}
		})

		c := xmlrpc.NewClient("localhost:9909")

		var res ResponseGetBusInfo
		err = c.Do("shutdown", RequestGetBusInfo{CallerID: "mycaller"}, &res)
		require.Error(t, err)
	})
}
