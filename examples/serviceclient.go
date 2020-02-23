// +build ignore

package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
)

// define a custom service.
// unlike the standard library, a .srv file is not needed.
// two structure definitions are enough, one for the request
// and the other for the response.

type TestServiceReq struct {
	A msgs.Float64
	B msgs.String
}

type TestServiceRes struct {
	C msgs.Float64
}

func main() {
	// create a node with given name and linked to given master.
	// master can be reached with an ip or hostname.
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/goroslib",
		MasterHost: "127.0.0.1",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a service client
	sc, err := goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node:    n,
		Service: "/test_srv",
		Req:     &TestServiceReq{},
		Res:     &TestServiceRes{},
	})
	if err != nil {
		panic(err)
	}
	defer sc.Close()

	// send a request and wait for a response
	req := TestServiceReq{
		A: 123,
		B: "456",
	}
	res := TestServiceRes{}
	err = sc.Call(&req, &res)
	if err != nil {
		panic(err)
	}

	fmt.Println("response:", res)
}
