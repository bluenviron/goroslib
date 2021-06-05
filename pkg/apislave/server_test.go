package apislave

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

func TestServer(t *testing.T) {
	s, err := NewServer("127.0.0.1:9999")
	require.NoError(t, err)
	defer s.Close()

	go s.Serve(func(req Request) Response {
		require.Equal(t, &RequestGetPid{CallerID: "mycaller"}, req)
		return ResponseGetPid{Code: 1, Pid: 123}
	})

	c := xmlrpc.NewClient("127.0.0.1:9999")

	var res ResponseGetPid
	err = c.Do("getPid", RequestGetPid{CallerID: "mycaller"}, &res)
	require.NoError(t, err)
	require.Equal(t, ResponseGetPid{Code: 1, Pid: 123}, res)
}
