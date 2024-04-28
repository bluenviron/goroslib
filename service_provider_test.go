package goroslib

import (
	"net"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_srvs"
	"github.com/bluenviron/goroslib/v2/pkg/protocommon"
	"github.com/bluenviron/goroslib/v2/pkg/prototcp"
	"github.com/bluenviron/goroslib/v2/pkg/serviceproc"
)

func TestServiceProviderRegister(t *testing.T) {
	m := newContainerMaster(t)
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
		Callback: func(req *TestServiceReq) (*TestServiceRes, bool) {
			c := float64(0)
			if req.A == 123 && req.B == "456" {
				c = 123
			}

			return &TestServiceRes{C: c}, true
		},
	})
	require.NoError(t, err)

	// test registration

	time.Sleep(1 * time.Second)

	services, err := n.MasterGetServices()
	require.NoError(t, err)

	srv, ok := services["/myns/test_srv"]
	require.Equal(t, true, ok)

	_, ok = srv.Providers["/myns/goroslib"]
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
	m := newContainerMaster(t)
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
		Callback: func(_ *std_srvs.SetBoolReq) (*std_srvs.SetBoolRes, bool) {
			return &std_srvs.SetBoolRes{
				Success: true,
				Message: "ok",
			}, true
		},
	})
	require.NoError(t, err)
	defer sp.Close()

	cc := newContainer(t, "rosservice-info", m.IP())

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
			m := newContainerMaster(t)
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
				var sp *ServiceProvider
				sp, err = NewServiceProvider(ServiceProviderConf{
					Node: nsp,
					Name: "test_srv",
					Srv:  &TestService{},
					Callback: func(req *TestServiceReq) (*TestServiceRes, bool) {
						c := float64(0)
						if req.A == 123 && req.B == "456" {
							c = 123
						}

						return &TestServiceRes{C: c}, true
					},
				})
				require.NoError(t, err)
				defer sp.Close()

			case "rosservice call":
				var sp *ServiceProvider
				sp, err = NewServiceProvider(ServiceProviderConf{
					Node: nsp,
					Name: "test_srv",
					Srv:  &std_srvs.SetBool{},
					Callback: func(_ *std_srvs.SetBoolReq) (*std_srvs.SetBoolRes, bool) {
						return &std_srvs.SetBoolRes{
							Success: true,
							Message: "ok",
						}, true
					},
				})
				require.NoError(t, err)
				defer sp.Close()
			}

			switch client {
			case "cpp":
				cc := newContainer(t, "node-serviceclient", m.IP())
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

				ur, err := nsc.apiMasterClient.LookupService(
					nsc.absoluteTopicName("test_srv"))
				require.NoError(t, err)

				address, err := urlToAddress(ur)
				require.NoError(t, err)

				for i := 0; i < 2; i++ { // test two connections with the same caller ID at once
					nconn, err := net.Dial("tcp", address)
					require.NoError(t, err)
					defer nconn.Close()
					tconn := prototcp.NewConn(nconn)

					srvMD5, err := serviceproc.MD5(TestService{})
					require.NoError(t, err)

					err = tconn.WriteHeader(&prototcp.HeaderServiceClient{
						Callerid:   nsc.absoluteName(),
						Md5sum:     srvMD5,
						Persistent: 1,
						Service:    nsc.absoluteTopicName("test_srv"),
					})
					require.NoError(t, err)

					raw, err := tconn.ReadHeaderRaw()
					require.NoError(t, err)

					_, ok := raw["error"]
					require.Equal(t, false, ok)

					var outHeader prototcp.HeaderServiceProvider
					err = protocommon.HeaderDecode(raw, &outHeader)
					require.NoError(t, err)
					require.Equal(t, srvMD5, outHeader.Md5sum)

					err = tconn.WriteMessage(&TestServiceReq{
						A: 123,
						B: "456",
					})
					require.NoError(t, err)

					var res TestServiceRes
					state, err := tconn.ReadServiceResponse(&res)
					require.NoError(t, err)
					require.Equal(t, true, state)
					require.Equal(t, TestServiceRes{C: 123}, res)
				}

			case "rosservice call":
				cc := newContainer(t, "rosservice-call", m.IP())
				require.Equal(t, "success: True\n"+
					"message: \"ok\"\n", cc.waitOutput())
			}
		})
	}
}
