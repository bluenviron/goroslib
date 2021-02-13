package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
)

func onService(req *std_srvs.SetBoolReq) *std_srvs.SetBoolRes {
	fmt.Println("request:", req)
	return &std_srvs.SetBoolRes{
		Success: true,
		Message: "test message",
	}
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
		Srv:      &std_srvs.SetBool{},
		Callback: onService,
	})
	if err != nil {
		panic(err)
	}
	defer sp.Close()

	// freeze main loop
	select {}
}
