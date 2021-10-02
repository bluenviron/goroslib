package goroslib

import (
	"sort"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

func TestNodeMasterGetNodes(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	p := newContainer(t, "node-gen", m.IP())
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

	res, err := n1.MasterGetNodes()
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

	res, err = n1.MasterGetNodes()
	require.NoError(t, err)

	require.Equal(t, 3, len(res))
}

func TestNodeMasterGetMachines(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	p := newContainer(t, "node-gen", m.IP())
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.MasterGetMachines()
	require.NoError(t, err)

	require.Equal(t, 3, len(res))
}

func TestNodeMasterGetTopics(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	p := newContainer(t, "node-pub", m.IP())
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.MasterGetTopics()
	require.NoError(t, err)

	topic, ok := res["/myns/test_topic"]
	require.True(t, ok)

	require.Equal(t, "nodepub/Mymsg", topic.Type)

	require.Equal(t, map[string]struct{}{"/myns/nodepub": {}}, topic.Publishers)
}

func TestNodeMasterGetServices(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	p := newContainer(t, "node-serviceprovider", m.IP())
	defer p.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	res, err := n.MasterGetServices()
	require.NoError(t, err)

	srv, ok := res["/myns/test_srv"]
	require.True(t, ok)

	require.Equal(t, map[string]struct{}{"/myns/nodeserviceprovider": {}}, srv.Providers)
}

func TestNodeNodeGetConns(t *testing.T) {
	for _, node := range []string{
		"cpp",
		"go",
	} {
		t.Run(node, func(t *testing.T) {
			m := newContainerMaster(t)
			defer m.close()

			switch node {
			case "cpp":
				en1 := newContainer(t, "node-businfo1", m.IP())
				defer en1.close()

				en2 := newContainer(t, "node-businfo2", m.IP())
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

			infos1, err := n.NodeGetConns("nodebusinfo1")
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

			infos2, err := n.NodeGetConns("nodebusinfo2")
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
