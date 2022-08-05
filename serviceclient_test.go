package goroslib

import (
	"context"
	"sync"
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

type TestService struct {
	TestServiceReq
	TestServiceRes
}

func TestServiceClientRequestAfterProvider(t *testing.T) {
	for _, provider := range []string{
		"cpp",
		"go",
	} {
		t.Run(provider, func(t *testing.T) {
			m := newContainerMaster(t)
			defer m.close()

			var p *container
			var nsp *Node
			var sp *ServiceProvider

			switch provider {
			case "cpp":
				p = newContainer(t, "node-serviceprovider", m.IP())

			case "go":
				var err error
				nsp, err = NewNode(NodeConf{
					Namespace:     "/myns",
					Name:          "goroslib_sp",
					MasterAddress: m.IP() + ":11311",
				})
				require.NoError(t, err)

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
			}

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			sc, err := NewServiceClient(ServiceClientConf{
				Node: n,
				Name: "test_srv",
				Srv:  &TestService{},
			})
			require.NoError(t, err)
			defer sc.Close()

			var wg sync.WaitGroup
			wg.Add(2)

			for i := 0; i < 2; i++ {
				go func() {
					defer wg.Done()

					req := TestServiceReq{
						A: 123,
						B: "456",
					}
					var res TestServiceRes
					err := sc.Call(&req, &res)
					require.NoError(t, err)

					expected := TestServiceRes{C: 123}
					require.Equal(t, expected, res)
				}()
			}

			wg.Wait()

			switch provider {
			case "cpp":
				p.close()

			case "go":
				sp.Close()
				nsp.Close()
			}

			req := TestServiceReq{
				A: 123,
				B: "456",
			}
			var res TestServiceRes
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
			m := newContainerMaster(t)
			defer m.close()

			n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

			sc, err := NewServiceClient(ServiceClientConf{
				Node: n,
				Name: "test_srv",
				Srv:  &TestService{},
			})
			require.NoError(t, err)
			defer sc.Close()

			req := TestServiceReq{
				A: 123,
				B: "456",
			}
			var res TestServiceRes
			err = sc.Call(&req, &res)
			require.Error(t, err)

			switch provider {
			case "cpp":
				p := newContainer(t, "node-serviceprovider", m.IP())
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
			}

			var wg sync.WaitGroup
			wg.Add(2)

			for i := 0; i < 2; i++ {
				go func() {
					defer wg.Done()

					req := TestServiceReq{
						A: 123,
						B: "456",
					}
					var res TestServiceRes
					err := sc.Call(&req, &res)
					require.NoError(t, err)

					expected := TestServiceRes{C: 123}
					require.Equal(t, expected, res)
				}()
			}

			wg.Wait()
		})
	}
}

func TestServiceClientContext(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	nsp, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib_sp",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer nsp.Close()

	spTerminate := make(chan struct{})
	clientCreated := make(chan struct{})
	sp, err := NewServiceProvider(ServiceProviderConf{
		Node: nsp,
		Name: "test_srv",
		Srv:  &TestService{},
		Callback: func(req *TestServiceReq) (*TestServiceRes, bool) {
			<-spTerminate
			return &TestServiceRes{C: 123}, true
		},
		onClient: func() {
			close(clientCreated)
		},
	})
	require.NoError(t, err)
	defer sp.Close()
	defer close(spTerminate)

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	sc, err := NewServiceClient(ServiceClientConf{
		Node: n,
		Name: "test_srv",
		Srv:  &TestService{},
	})
	require.NoError(t, err)
	defer sc.Close()

	ctx, ctxCancel := context.WithCancel(context.Background())

	done := make(chan struct{})
	go func() {
		err := sc.CallContext(ctx, &TestServiceReq{
			A: 123,
			B: "456",
		}, &TestServiceRes{})
		require.Error(t, err)
		close(done)
	}()

	<-clientCreated
	ctxCancel()
	<-done
}

func TestServiceClientErrors(t *testing.T) {
	m := newContainerMaster(t)
	defer m.close()

	n, err := NewNode(NodeConf{
		Namespace:     "/myns",
		Name:          "goroslib_sp",
		MasterAddress: m.IP() + ":11311",
	})
	require.NoError(t, err)
	defer n.Close()

	t.Run("missing node", func(t *testing.T) {
		_, err := NewServiceClient(ServiceClientConf{})
		require.Error(t, err)
	})

	t.Run("missing name", func(t *testing.T) {
		_, err := NewServiceClient(ServiceClientConf{
			Node: n,
		})
		require.Error(t, err)
	})

	t.Run("missing srv", func(t *testing.T) {
		_, err := NewServiceClient(ServiceClientConf{
			Node: n,
			Name: "test_srv",
		})
		require.Error(t, err)
	})
}
