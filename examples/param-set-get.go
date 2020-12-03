// +build ignore

package main

import (
	"fmt"

	"github.com/aler9/goroslib"
)

func main() {
	// create a node and connects to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_params",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// set a param of type int
	err = n.SetParamInt("myparam", 123)
	if err != nil {
		panic(err)
	}

	// get the same param
	val, err := n.GetParamInt("myparam")
	if err != nil {
		panic(err)
	}

	fmt.Println("param:", val)
}
