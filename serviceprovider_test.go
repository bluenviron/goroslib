package goroslib

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
)

func TestServiceProviderRegister(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	sp, err := NewServiceProvider(ServiceProviderConf{
		Node: n,
		Name: "test_srv",
		Srv:  &TestService{},
		Callback: func(req *TestServiceReq) *TestServiceRes {
			c := float64(0)
			if req.A == 123 && req.B == "456" {
				c = 123
			}

			return &TestServiceRes{C: c}
		},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	services, err := n.MasterGetServices()
	require.NoError(t, err)

	service, ok := services["/myns/test_srv"]
	require.Equal(t, true, ok)

	_, ok = service.Providers["/myns/goroslib"]
	require.Equal(t, true, ok)

	// test un-registration

	sp.Close()
	time.Sleep(1 * time.Second)

	services, err = n.MasterGetServices()
	require.NoError(t, err)

	_, ok = services["/myns/test_srv"]
	require.Equal(t, false, ok)
}

func TestServiceProviderInfo(t *testing.T) {
	m, err := newContainerMaster()
	require.NoError(t, err)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	sp, err := NewServiceProvider(ServiceProviderConf{
		Node: n,
		Name: "test_srv",
		Srv:  &std_srvs.SetBool{},
		Callback: func(req *std_srvs.SetBoolReq) *std_srvs.SetBoolRes {
			return &std_srvs.SetBoolRes{
				Success: true,
				Message: "ok",
			}
		},
	})
	require.NoError(t, err)
	defer sp.Close()

	cc, err := newContainer("rosservice-info", m.IP())
	require.NoError(t, err)

	require.Regexp(t, "Node: /myns/goroslib\n"+
		"URI: rosrpc://172.17.0.[0-9]:[0-9]+\n"+
		"Type: std_srvs/SetBool\n"+
		"Args: data\n", cc.waitOutput())
}

func TestServiceProviderResponse(t *testing.T) {
	for _, client := range []string{
		"cpp",
		"go",
		"rosservice call",
	} {
		t.Run(client, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			nsp, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib_sp",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer nsp.Close()

			switch client {
			case "cpp", "go":
				sp, err := NewServiceProvider(ServiceProviderConf{
					Node: nsp,
					Name: "test_srv",
					Srv:  &TestService{},
					Callback: func(req *TestServiceReq) *TestServiceRes {
						c := float64(0)
						if req.A == 123 && req.B == "456" {
							c = 123
						}
						return &TestServiceRes{C: c}
					},
				})
				require.NoError(t, err)
				defer sp.Close()

			case "rosservice call":
				sp, err := NewServiceProvider(ServiceProviderConf{
					Node: nsp,
					Name: "test_srv",
					Srv:  &std_srvs.SetBool{},
					Callback: func(req *std_srvs.SetBoolReq) *std_srvs.SetBoolRes {
						return &std_srvs.SetBoolRes{
							Success: true,
							Message: "ok",
						}
					},
				})
				require.NoError(t, err)
				defer sp.Close()
			}

			switch client {
			case "cpp":
				cc, err := newContainer("node-serviceclient", m.IP())
				require.NoError(t, err)
				require.Equal(t, "123.000000\n", cc.waitOutput())

			case "go":
				nsc, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_sc",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer nsc.Close()

				sc, err := NewServiceClient(ServiceClientConf{
					Node: nsc,
					Name: "test_srv",
					Srv:  &TestService{},
				})
				require.NoError(t, err)
				defer sc.Close()

				req := TestServiceReq{
					A: 123,
					B: "456",
				}
				res := TestServiceRes{}
				err = sc.Call(&req, &res)
				require.NoError(t, err)

				expected := TestServiceRes{C: 123}
				require.Equal(t, expected, res)

			case "rosservice call":
				cc, err := newContainer("rosservice-call", m.IP())
				require.NoError(t, err)
				require.Equal(t, "success: True\n"+
					"message: \"ok\"\n", cc.waitOutput())
			}
		})
	}
}
