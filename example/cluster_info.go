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
		Name:       "/goroslib",
		MasterHost: "127.0.0.1",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// get all nodes linked to master
	nodes, err := n.GetNodes()
	if err != nil {
		panic(err)
	}

	fmt.Println("nodes:", nodes)

	// get all machines that are hosting nodes linked to master
	machines, err := n.GetMachines()
	if err != nil {
		panic(err)
	}

	fmt.Println("machines:", machines)

	// get all topics
	topics, err := n.GetTopics()
	if err != nil {
		panic(err)
	}

	fmt.Println("topics:", topics)

	// get all services
	services, err := n.GetServices()
	if err != nil {
		panic(err)
	}

	fmt.Println("services:", services)
}
