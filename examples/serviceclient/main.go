// Package main contains an example.
package main

import (
	"log"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

// define a custom service.
// unlike the standard library, a .srv file is not needed.

type TestServiceReq struct { //nolint:revive
	A float64
	B string
}

type TestServiceRes struct { //nolint:revive
	C float64
}

type TestService struct { //nolint:revive
	msg.Package `ros:"my_package"`
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

	log.Println("response:", res)
}
