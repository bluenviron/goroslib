package goroslib

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/msg"
)

type TestServiceReq struct {
	A msg.Float64
	B msg.String
}

type TestServiceRes struct {
	C msg.Float64
}

func TestServiceClient(t *testing.T) {
	m, err := newCntMaster()
	require.NoError(t, err)
	defer m.close()

	p, err := newCntNodeserviceprovider(m.Ip())
	require.NoError(t, err)
	defer p.close()

	n, err := NewNode(NodeConf{
		Name:       "/goroslib",
		MasterHost: m.Ip(),
	})
	require.NoError(t, err)
	defer n.Close()

	sc, err := NewServiceClient(ServiceClientConf{
		Node:    n,
		Service: "/test_srv",
		Req:     &TestServiceReq{},
		Res:     &TestServiceRes{},
	})
	require.NoError(t, err)
	defer sc.Close()

	for i := 0; i < 2; i++ {
		req := TestServiceReq{
			A: 123,
			B: "456",
		}
		res := TestServiceRes{}
		err = sc.Call(&req, &res)
		require.NoError(t, err)

		expected := TestServiceRes{
			C: 123,
		}
		require.Equal(t, expected, res)
	}
}
