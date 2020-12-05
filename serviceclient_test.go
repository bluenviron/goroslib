package goroslib

import (
	"testing"

	"github.com/stretchr/testify/require"
)

type TestServiceReq struct {
	A float64
	B string
}

type TestServiceRes struct {
	C float64
}

func TestServiceClientRequestAfterProvider(t *testing.T) {
	for _, provider := range []string{
		"cpp",
		"go",
	} {
		t.Run(provider, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

			var p *container
			var nsp *Node
			var sp *ServiceProvider

			switch provider {
			case "cpp":
				var err error
				p, err = newContainer("node-serviceprovider", m.IP())
				require.NoError(t, err)

			case "go":
				nsp, err = NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_sp",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)

				sp, err = NewServiceProvider(ServiceProviderConf{
					Node:    nsp,
					Service: "test_srv",
					Callback: func(req *TestServiceReq) *TestServiceRes {
						c := float64(0)
						if req.A == 123 && req.B == "456" {
							c = 123
						}
						return &TestServiceRes{C: c}
					},
				})
				require.NoError(t, err)
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			sc, err := NewServiceClient(ServiceClientConf{
				Node:    n,
				Service: "test_srv",
				Req:     &TestServiceReq{},
				Res:     &TestServiceRes{},
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

			switch provider {
			case "cpp":
				p.close()

			case "go":
				sp.Close()
				nsp.Close()
			}

			req = TestServiceReq{
				A: 123,
				B: "456",
			}
			res = TestServiceRes{}
			err = sc.Call(&req, &res)
			require.Error(t, err)
		})
	}
}

func TestServiceClientRequestBeforeProvider(t *testing.T) {
	for _, provider := range []string{
		"cpp",
		"go",
	} {
		t.Run(provider, func(t *testing.T) {
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

			sc, err := NewServiceClient(ServiceClientConf{
				Node:    n,
				Service: "test_srv",
				Req:     &TestServiceReq{},
				Res:     &TestServiceRes{},
			})
			require.NoError(t, err)
			defer sc.Close()

			req := TestServiceReq{
				A: 123,
				B: "456",
			}
			res := TestServiceRes{}
			err = sc.Call(&req, &res)
			require.Error(t, err)

			switch provider {
			case "cpp":
				p, err := newContainer("node-serviceprovider", m.IP())
				require.NoError(t, err)
				defer p.close()

			case "go":
				nsp, err := NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_sp",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)
				defer nsp.Close()

				sp, err := NewServiceProvider(ServiceProviderConf{
					Node:    nsp,
					Service: "test_srv",
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
			}

			req = TestServiceReq{
				A: 123,
				B: "456",
			}
			res = TestServiceRes{}
			err = sc.Call(&req, &res)
			require.NoError(t, err)

			expected := TestServiceRes{C: 123}
			require.Equal(t, expected, res)
		})
	}
}
