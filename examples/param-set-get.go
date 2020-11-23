// +build ignore

package main

import (
	"fmt"

	"github.com/aler9/goroslib"
)

func main() {
	// create a node with given name and linked to given master.
	// master can be reached with an ip or hostname.
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "goroslib_params",
		MasterHost: "127.0.0.1",
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
