package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/pkg/msgs/std_srvs"
)

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
		Srv:  &std_srvs.SetBool{},
	})
	if err != nil {
		panic(err)
	}
	defer sc.Close()

	// send a request and wait for a response
	req := std_srvs.SetBoolReq{
		Data: true,
	}
	res := std_srvs.SetBoolRes{}
	err = sc.Call(&req, &res)
	if err != nil {
		panic(err)
	}

	fmt.Println("response:", res)
}
