// +build ignore

package main

import (
	"fmt"

	"github.com/aler9/goroslib"
)

// define a custom service.
// unlike the standard library, a .srv file is not needed.

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

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_sc",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a service client
	sc, err := goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: n,
		Name: "test_srv",
		Srv:  &TestService{},
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
