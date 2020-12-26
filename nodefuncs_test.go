package goroslib

import (
	"sort"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

func TestNodeGetNodes(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newContainer("node-gen", m.IP())
	require.NoError(t, err)
	defer p.close()

	n1, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n1.Close()

	n2, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib2",
		MasterAddress: m.IP() + ":11311",
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

	p, err := newContainer("node-gen", m.IP())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
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

	p, err := newContainer("node-pub", m.IP())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
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

	p, err := newContainer("node-serviceprovider", m.IP())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
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
				p, err := newContainer("node-gen", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				n1, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "nodegen",
					MasterAddress: m.IP() + ":11311",
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
				MasterAddress: m.IP() + ":11311",
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
				p, err := newContainer("node-gen", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				n1, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "nodegen",
					MasterAddress: m.IP() + ":11311",
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
					Node: n1,
					Name: "test_srv",
					Srv:  &TestService{},
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
				MasterAddress: m.IP() + ":11311",
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

func TestNodeGetNodeConns(t *testing.T) {
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
				en1, err := newContainer("node-businfo1", m.IP())
				require.NoError(t, err)
				defer en1.close()

				en2, err := newContainer("node-businfo2", m.IP())
				require.NoError(t, err)
				defer en2.close()

				time.Sleep(1 * time.Second)

			case "go":
				en1, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "nodebusinfo1",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer en1.Close()

				pub, err := NewPublisher(PublisherConf{
					Node:  en1,
					Topic: "test_topic",
					Msg:   &std_msgs.String{},
				})
				require.NoError(t, err)
				defer pub.Close()

				en2, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "nodebusinfo2",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer en2.Close()

				sub, err := NewSubscriber(SubscriberConf{
					Node:     en2,
					Topic:    "test_topic",
					Protocol: UDP,
					Callback: func(msg *std_msgs.String) {
					},
				})
				require.NoError(t, err)
				defer sub.Close()

				time.Sleep(1 * time.Second)
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			infos1, err := n.GetNodeConns("nodebusinfo1")
			require.NoError(t, err)

			sort.Slice(infos1, func(a, b int) bool {
				return infos1[a].Topic < infos1[b].Topic
			})

			require.Equal(t, []InfoConnection{
				{
					ID:        infos1[0].ID,
					To:        "/myns/nodebusinfo2",
					Direction: 'o',
					Transport: "UDPROS",
					Topic:     "/myns/test_topic",
					Connected: true,
				},
				{
					ID:        infos1[1].ID,
					To:        "/rosout",
					Direction: 'o',
					Transport: "TCPROS",
					Topic:     "/rosout",
					Connected: true,
				},
			}, infos1)

			infos2, err := n.GetNodeConns("nodebusinfo2")
			require.NoError(t, err)

			sort.Slice(infos2, func(a, b int) bool {
				return infos2[a].Topic < infos2[b].Topic
			})

			require.Regexp(t, `^http://`, infos2[0].To)

			require.Equal(t, []InfoConnection{
				{
					ID:        infos2[0].ID,
					To:        infos2[0].To,
					Direction: 'i',
					Transport: "UDPROS",
					Topic:     "/myns/test_topic",
					Connected: true,
				},
				{
					ID:        infos2[1].ID,
					To:        "/rosout",
					Direction: 'o',
					Transport: "TCPROS",
					Topic:     "/rosout",
					Connected: true,
				},
			}, infos2)
		})
	}
}
