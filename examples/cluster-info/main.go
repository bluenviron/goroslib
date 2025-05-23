// Package main contains an example.
package main

import (
	"log"

	"github.com/bluenviron/goroslib/v2"
)

func main() {
	// create a node and connect to the master
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:          "goroslib_info",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer n.Close()

	// get all nodes linked to master
	nodes, err := n.MasterGetNodes()
	if err != nil {
		panic(err)
	}

	log.Println("nodes:", nodes)

	// get all machines that are hosting nodes linked to master
	machines, err := n.MasterGetMachines()
	if err != nil {
		panic(err)
	}

	log.Println("machines:", machines)

	// get all topics
	topics, err := n.MasterGetTopics()
	if err != nil {
		panic(err)
	}

	log.Println("topics:", topics)

	// get all services
	services, err := n.MasterGetServices()
	if err != nil {
		panic(err)
	}

	log.Println("services:", services)
}
