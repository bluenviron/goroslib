package apimaster

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

func TestClient(t *testing.T) {
	s, err := xmlrpc.NewServer("localhost:9997")
	require.NoError(t, err)
	defer s.Close()

	go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
		switch raw.Method {
		case "getPublishedTopics":
			return ResponseGetPublishedTopics{Code: 1, Topics: [][]string{{"mytopic"}}}

		case "getSystemState":
			return ResponseGetSystemState{
				Code:  1,
				State: SystemState{PublishedTopics: []SystemStateEntry{{Name: "myname", Nodes: []string{"mynode"}}}},
			}

		case "getTopicTypes":
			return ResponseGetTopicTypes{Code: 1, Types: []TopicType{{Name: "myname", Type: "mytype"}}}

		case "getUri":
			return ResponseGetURI{Code: 1, MasterURI: "myuri"}

		case "lookupNode":
			return ResponseLookup{Code: 1, URL: "myurl"}

		case "lookupService":
			return ResponseLookup{Code: 1, URL: "myurl"}

		case "registerSubscriber":
			return ResponseRegister{Code: 1, URIs: []string{"myurl"}}

		case "registerPublisher":
			return ResponseRegister{Code: 1, URIs: []string{"myurl"}}

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
		require.Equal(t, [][]string{{"mytopic"}}, res)
	}()

	func() {
		res, err := c.GetSystemState()
		require.NoError(t, err)
		require.Equal(t, &SystemState{PublishedTopics: []SystemStateEntry{{Name: "myname", Nodes: []string{"mynode"}}}}, res)
	}()

	func() {
		res, err := c.GetTopicTypes()
		require.NoError(t, err)
		require.Equal(t, []TopicType{{Name: "myname", Type: "mytype"}}, res)
	}()

	func() {
		res, err := c.GetURI()
		require.NoError(t, err)
		require.Equal(t, "myuri", res)
	}()

	func() {
		res, err := c.LookupNode("mynode")
		require.NoError(t, err)
		require.Equal(t, "myurl", res)
	}()

	func() {
		res, err := c.LookupService("myservice")
		require.NoError(t, err)
		require.Equal(t, "myurl", res)
	}()

	func() {
		res, err := c.RegisterSubscriber("mytopic", "mytype", "myurl")
		require.NoError(t, err)
		require.Equal(t, []string{"myurl"}, res)
	}()

	func() {
		res, err := c.RegisterPublisher("mytopic", "mytype", "myurl")
		require.NoError(t, err)
		require.Equal(t, []string{"myurl"}, res)
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

func TestClientErrors(t *testing.T) {
	c := NewClient("localhost:9997", "test")

	t.Run("no server", func(t *testing.T) {
		func() {
			_, err := c.GetPublishedTopics("mysubgraph")
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetSystemState()
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetTopicTypes()
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetURI()
			require.Error(t, err)
		}()

		func() {
			_, err := c.LookupNode("mynode")
			require.Error(t, err)
		}()

		func() {
			_, err := c.LookupService("myservice")
			require.Error(t, err)
		}()

		func() {
			_, err := c.RegisterSubscriber("mytopic", "mytype", "myurl")
			require.Error(t, err)
		}()

		func() {
			_, err := c.RegisterPublisher("mytopic", "mytype", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.UnregisterSubscriber("mytopic", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.UnregisterPublisher("mytopic", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.RegisterService("myservice", "serviceurl", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.UnregisterService("myservice", "serviceurl")
			require.Error(t, err)
		}()
	})

	t.Run("server error", func(t *testing.T) {
		s, err := xmlrpc.NewServer("localhost:9997")
		require.NoError(t, err)
		defer s.Close()

		go s.Serve(func(raw *xmlrpc.RequestRaw) interface{} {
			switch raw.Method {
			case "getPublishedTopics":
				return ResponseGetPublishedTopics{Code: 0, Topics: [][]string{{"mytopic"}}}

			case "getSystemState":
				return ResponseGetSystemState{
					Code:  0,
					State: SystemState{PublishedTopics: []SystemStateEntry{{Name: "myname", Nodes: []string{"mynode"}}}},
				}

			case "getTopicTypes":
				return ResponseGetTopicTypes{Code: 0, Types: []TopicType{{Name: "myname", Type: "mytype"}}}

			case "getUri":
				return ResponseGetURI{Code: 0, MasterURI: "myuri"}

			case "lookupNode":
				return ResponseLookup{Code: 0, URL: "myurl"}

			case "lookupService":
				return ResponseLookup{Code: 0, URL: "myurl"}

			case "registerSubscriber":
				return ResponseRegister{Code: 0, URIs: []string{"myurl"}}

			case "registerPublisher":
				return ResponseRegister{Code: 0, URIs: []string{"myurl"}}

			case "unregisterSubscriber":
				return ResponseUnregister{Code: 0, NumUnregistered: 1}

			case "unregisterPublisher":
				return ResponseUnregister{Code: 0, NumUnregistered: 1}

			case "registerService":
				return ResponseRegisterService{Code: 0}

			case "unregisterService":
				return ResponseServiceUnregister{Code: 0, NumUnregistered: 1}
			}
			return xmlrpc.ErrorRes{}
		})

		func() {
			_, err := c.GetPublishedTopics("mysubgraph")
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetSystemState()
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetTopicTypes()
			require.Error(t, err)
		}()

		func() {
			_, err := c.GetURI()
			require.Error(t, err)
		}()

		func() {
			_, err := c.LookupNode("mynode")
			require.Error(t, err)
		}()

		func() {
			_, err := c.LookupService("myservice")
			require.Error(t, err)
		}()

		func() {
			_, err := c.RegisterSubscriber("mytopic", "mytype", "myurl")
			require.Error(t, err)
		}()

		func() {
			_, err := c.RegisterPublisher("mytopic", "mytype", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.UnregisterSubscriber("mytopic", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.UnregisterPublisher("mytopic", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.RegisterService("myservice", "serviceurl", "myurl")
			require.Error(t, err)
		}()

		func() {
			err := c.UnregisterService("myservice", "serviceurl")
			require.Error(t, err)
		}()
	})
}
