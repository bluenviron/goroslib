package apislave

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

func TestClient(t *testing.T) {
	s, err := xmlrpc.NewServer("localhost:9905")
	require.NoError(t, err)
	defer s.Close()

	go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
		switch raw.Method {
		case "getPid":
			return ResponseGetPid{Code: 1, Pid: 123}

		case "shutdown":
			return ResponseShutdown{Code: 1}

		case "requestTopic":
			return ResponseRequestTopic{Code: 1, Protocol: []interface{}{"myproto"}}

		case "getBusInfo":
			return ResponseGetBusInfo{Code: 1}

		case "getPublications":
			return ResponseGetPublications{Code: 1, TopicList: [][]string{{"mytopic"}}}

		case "publisherUpdate":
			return ResponsePublisherUpdate{Code: 1}
		}
		return xmlrpc.ErrorRes{}
	})

	c := NewClient("localhost:9905", "test")

	func() {
		res, err := c.GetPid()
		require.NoError(t, err)
		require.Equal(t, 123, res)
	}()

	func() {
		err := c.Shutdown("myreason")
		require.NoError(t, err)
	}()

	func() {
		res, err := c.RequestTopic("mytopic", [][]interface{}{{"testing"}})
		require.NoError(t, err)
		require.Equal(t, []interface{}{"myproto"}, res)
	}()

	func() {
		res, err := c.GetBusInfo()
		require.NoError(t, err)
		require.Equal(t, [][]interface{}(nil), res)
	}()

	func() {
		res, err := c.GetPublications()
		require.NoError(t, err)
		require.Equal(t, [][]string{{"mytopic"}}, res)
	}()

	func() {
		err := c.PublisherUpdate("mytopic", []string{"myurl1", "myurl2"})
		require.NoError(t, err)
	}()
}

func TestClientErrors(t *testing.T) {
	c := NewClient("localhost:9905", "test")

	t.Run("no server", func(t *testing.T) {
		func() {
			_, err := c.GetPid()
			require.Error(t, err)
		}()

		err := c.Shutdown("myreason")
		require.Error(t, err)

		_, err = c.RequestTopic("mytopic", [][]interface{}{{"testing"}})
		require.Error(t, err)

		_, err = c.GetBusInfo()
		require.Error(t, err)

		_, err = c.GetPublications()
		require.Error(t, err)

		err = c.PublisherUpdate("mytopic", []string{"myurl1", "myurl2"})
		require.Error(t, err)
	})

	t.Run("server error", func(t *testing.T) {
		s, err := xmlrpc.NewServer("localhost:9905")
		require.NoError(t, err)
		defer s.Close()

		go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
			switch raw.Method {
			case "getPid":
				return ResponseGetPid{Code: 0, Pid: 123}

			case "shutdown":
				return ResponseShutdown{Code: 0}

			case "requestTopic":
				return ResponseRequestTopic{Code: 0, Protocol: []interface{}{"myproto"}}

			case "getBusInfo":
				return ResponseGetBusInfo{Code: 0}

			case "getPublications":
				return ResponseGetPublications{Code: 0}

			case "publisherUpdate":
				return ResponsePublisherUpdate{Code: 0}
			}
			return xmlrpc.ErrorRes{}
		})

		_, err = c.GetPid()
		require.Error(t, err)

		err = c.Shutdown("myreason")
		require.Error(t, err)

		_, err = c.RequestTopic("mytopic", [][]interface{}{{"testing"}})
		require.Error(t, err)

		_, err = c.GetBusInfo()
		require.Error(t, err)

		_, err = c.GetPublications()
		require.Error(t, err)

		err = c.PublisherUpdate("mytopic", []string{"myurl1", "myurl2"})
		require.Error(t, err)
	})
}
