package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestNodeNodePing(t *testing.T) {
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

			_, err = n.NodePing("nodegen")
			require.NoError(t, err)
		})
	}
}

func TestNodeNodeKill(t *testing.T) {
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

			err = n.NodeKill("nodegen")
			require.NoError(t, err)

			time.Sleep(1 * time.Second)

			res, err := n.GetNodes()
			require.NoError(t, err)

			require.Equal(t, 2, len(res))
		})
	}
}
