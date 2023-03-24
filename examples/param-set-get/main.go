package main

import (
	"fmt"

	"github.com/bluenviron/goroslib/v2"
)

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_params",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// set a param of type int
	err = n.ParamSetInt("myparam", 123)
	if err != nil {
		panic(err)
	}

	// get the same param
	val, err := n.ParamGetInt("myparam")
	if err != nil {
		panic(err)
	}

	fmt.Println("param:", val)
}
