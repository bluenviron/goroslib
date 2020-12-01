package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestNodeGetNodes(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n1, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.Ip() + ":11311",
	})
	require.NoError(t, err)
	defer n1.Close()

	n2, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib2",
		MasterAddress: m.Ip() + ":11311",
	})
	require.NoError(t, err)

	res, err := n1.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 4, len(res))

	node, ok := res["/myns/nodegen"]
	require.True(t, ok)

	_, ok = node.PublishedTopics["/rosout"]
	require.True(t, ok)

	_, ok = node.ProvidedServices["/myns/nodegen/set_logger_level"]
	require.True(t, ok)

	_, ok = node.ProvidedServices["/myns/nodegen/get_loggers"]
	require.True(t, ok)

	n2.Close()

	res, err = n1.GetNodes()
	require.NoError(t, err)

	require.Equal(t, 3, len(res))
}

func TestNodeGetMachines(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.Ip() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetMachines()
	require.NoError(t, err)

	require.Equal(t, 3, len(res))
}

func TestNodeGetTopics(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-pub", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.Ip() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetTopics()
	require.NoError(t, err)

	topic, ok := res["/myns/test_topic"]
	require.True(t, ok)

	require.Equal(t, "nodepub/Mymsg", topic.Type)

	require.Equal(t, map[string]struct{}{"/myns/nodepub": {}}, topic.Publishers)
}

func TestNodeGetServices(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-serviceprovider", m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.Ip() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.GetServices()
	require.NoError(t, err)

	service, ok := res["/myns/test_srv"]
	require.True(t, ok)

	require.Equal(t, map[string]struct{}{"/myns/nodeserviceprovider": {}}, service.Providers)
}

func TestNodePingNode(t *testing.T) {
	for _, node := range []string{
		"cpp",
		"go",
	} {
		t.Run(node, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch node {
			case "cpp":
				p, err := newContainer("node-gen", m.Ip())
				require.NoError(t, err)
				defer p.close()

			case "go":
				n1, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "nodegen",
					MasterAddress: m.Ip() + ":11311",
				})
				require.NoError(t, err)
				defer n1.Close()

				pub, err := NewPublisher(PublisherConf{
					Node:  n1,
					Topic: "test_topic",
					Msg:   &TestMessage{},
				})
				require.NoError(t, err)
				defer pub.Close()
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.Ip() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			_, err = n.PingNode("nodegen")
			require.NoError(t, err)
		})
	}
}

func TestNodeKillNode(t *testing.T) {
	for _, node := range []string{
		"cpp",
		"go",
	} {
		t.Run(node, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch node {
			case "cpp":
				p, err := newContainer("node-gen", m.Ip())
				require.NoError(t, err)
				defer p.close()

			case "go":
				n1, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "nodegen",
					MasterAddress: m.Ip() + ":11311",
				})
				require.NoError(t, err)
				defer n1.Close()

				pub, err := NewPublisher(PublisherConf{
					Node:  n1,
					Topic: "test_topic",
					Msg:   &TestMessage{},
				})
				require.NoError(t, err)
				defer pub.Close()

				sp, err := NewServiceProvider(ServiceProviderConf{
					Node:    n1,
					Service: "test_srv",
					Callback: func(req *TestServiceReq) *TestServiceRes {
						return &TestServiceRes{}
					},
				})
				require.NoError(t, err)
				defer sp.Close()
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.Ip() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			err = n.KillNode("nodegen")
			require.NoError(t, err)

			time.Sleep(1 * time.Second)

			res, err := n.GetNodes()
			require.NoError(t, err)

			require.Equal(t, 2, len(res))
		})
	}
}

func TestNodeGetParam(t *testing.T) {
	for _, lang := range []string{
		"cpp",
		"go",
	} {
		t.Run(lang, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			switch lang {
			case "cpp":
				p, err := newContainer("node-setparam", m.Ip())
				require.NoError(t, err)
				defer p.close()

			case "go":
				n, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_set",
					MasterAddress: m.Ip() + ":11311",
				})
				require.NoError(t, err)
				defer n.Close()

				err = n.SetParamBool("test_bool", true)
				require.NoError(t, err)

				err = n.SetParamInt("test_int", 123)
				require.NoError(t, err)

				err = n.SetParamString("test_string", "ABC")
				require.NoError(t, err)
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.Ip() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			resb, err := n.GetParamBool("test_bool")
			require.NoError(t, err)
			require.Equal(t, true, resb)

			resi, err := n.GetParamInt("test_int")
			require.NoError(t, err)
			require.Equal(t, 123, resi)

			ress, err := n.GetParamString("test_string")
			require.NoError(t, err)
			require.Equal(t, "ABC", ress)
		})
	}
}
