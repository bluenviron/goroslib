// Package main contains an example.
package main

import (
	"log"
	"os"
	"os/signal"

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

func onService(req *TestServiceReq) (*TestServiceRes, bool) {
	log.Println("request:", req)
	return &TestServiceRes{
		C: 123,
	}, true
}

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_sp",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// create a service provider
	sp, err := goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
		Node:     n,
		Name:     "test_srv",
		Srv:      &TestService{},
		Callback: onService,
	})
	if err != nil {
		panic(err)
	}
	defer sp.Close()

	// wait for CTRL-C
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	<-c
}
