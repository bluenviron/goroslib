package apimaster

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

func TestClient(t *testing.T) {
	s, err := xmlrpc.NewServer(9997)
	require.NoError(t, err)
	defer s.Close()

	go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
		switch raw.Method {
		case "getPublishedTopics":
			return ResponseGetPublishedTopics{Code: 1}

		case "getSystemState":
			return ResponseGetSystemState{Code: 1}

		case "getTopicTypes":
			return ResponseGetTopicTypes{Code: 1}

		case "getUri":
			return ResponseGetURI{Code: 1}

		case "lookupNode":
			return ResponseLookup{Code: 1}

		case "lookupService":
			return ResponseLookup{Code: 1}

		case "registerSubscriber":
			return ResponseRegister{Code: 1}

		case "registerPublisher":
			return ResponseRegister{Code: 1}

		case "unregisterSubscriber":
			return ResponseUnregister{Code: 1, NumUnregistered: 1}

		case "unregisterPublisher":
			return ResponseUnregister{Code: 1, NumUnregistered: 1}

		case "registerService":
			return ResponseRegisterService{Code: 1}

		case "unregisterService":
			return ResponseServiceUnregister{Code: 1, NumUnregistered: 1}
		}
		return xmlrpc.ErrorRes{}
	})

	c := NewClient("localhost:9997", "test")

	func() {
		res, err := c.GetPublishedTopics("mysubgraph")
		require.NoError(t, err)
		require.Equal(t, &ResponseGetPublishedTopics{Code: 1}, res)
	}()

	func() {
		res, err := c.GetSystemState()
		require.NoError(t, err)
		require.Equal(t, &ResponseGetSystemState{Code: 1}, res)
	}()

	func() {
		res, err := c.GetTopicTypes()
		require.NoError(t, err)
		require.Equal(t, &ResponseGetTopicTypes{Code: 1}, res)
	}()

	func() {
		res, err := c.GetURI()
		require.NoError(t, err)
		require.Equal(t, &ResponseGetURI{Code: 1}, res)
	}()

	func() {
		res, err := c.LookupNode("mynode")
		require.NoError(t, err)
		require.Equal(t, &ResponseLookup{Code: 1}, res)
	}()

	func() {
		res, err := c.LookupService("myservice")
		require.NoError(t, err)
		require.Equal(t, &ResponseLookup{Code: 1}, res)
	}()

	func() {
		res, err := c.RegisterSubscriber("mytopic", "mytype", "myurl")
		require.NoError(t, err)
		require.Equal(t, &ResponseRegister{Code: 1}, res)
	}()

	func() {
		res, err := c.RegisterPublisher("mytopic", "mytype", "myurl")
		require.NoError(t, err)
		require.Equal(t, &ResponseRegister{Code: 1}, res)
	}()

	func() {
		err := c.UnregisterSubscriber("mytopic", "myurl")
		require.NoError(t, err)
	}()

	func() {
		err := c.UnregisterPublisher("mytopic", "myurl")
		require.NoError(t, err)
	}()

	func() {
		err := c.RegisterService("myservice", "serviceurl", "myurl")
		require.NoError(t, err)
	}()

	func() {
		err := c.UnregisterService("myservice", "serviceurl")
		require.NoError(t, err)
	}()
}
