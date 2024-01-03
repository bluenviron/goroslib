package goroslib

import (
	"fmt"
	"net"
)

// InfoNode contains informations about a node.
type InfoNode struct {
	PublishedTopics  map[string]struct{}
	SubscribedTopics map[string]struct{}
	ProvidedServices map[string]struct{}
	Address          string
}

// MasterGetNodes returns all the nodes connected to the master.
func (n *Node) MasterGetNodes() (map[string]*InfoNode, error) {
	state, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, err
	}

	ret := make(map[string]*InfoNode)

	initEntry := func(node string) {
		if _, ok := ret[node]; !ok {
			ret[node] = &InfoNode{
				PublishedTopics:  make(map[string]struct{}),
				SubscribedTopics: make(map[string]struct{}),
				ProvidedServices: make(map[string]struct{}),
			}
		}
	}

	for _, entry := range state.PublishedTopics {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].PublishedTopics[entry.Name] = struct{}{}
		}
	}

	for _, entry := range state.SubscribedTopics {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].SubscribedTopics[entry.Name] = struct{}{}
		}
	}

	for _, entry := range state.ProvidedServices {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].ProvidedServices[entry.Name] = struct{}{}
		}
	}

	for nodeName, info := range ret {
		ur, err := n.apiMasterClient.LookupNode(nodeName)
		if err != nil {
			return nil, fmt.Errorf("lookupNode: %w", err)
		}

		address, err := urlToAddress(ur)
		if err != nil {
			return nil, err
		}

		info.Address = address
	}

	return ret, nil
}

// MasterGetMachines returns all the machines connected to the master with a node.
func (n *Node) MasterGetMachines() (map[string]struct{}, error) {
	// this is like its equivalent in python
	// https://docs.ros.org/noetic/api/rosnode/html/rosnode-pysrc.html#get_machines_by_nodes

	nodes, err := n.MasterGetNodes()
	if err != nil {
		return nil, err
	}

	ret := make(map[string]struct{})
	for _, info := range nodes {
		host, _, err := net.SplitHostPort(info.Address)
		if err != nil {
			continue
		}
		ret[host] = struct{}{}
	}

	return ret, nil
}

// InfoTopic contains informations about a topic.
type InfoTopic struct {
	Type        string
	Publishers  map[string]struct{}
	Subscribers map[string]struct{}
}

// MasterGetTopics returns all the topics published by nodes connected to the master.
func (n *Node) MasterGetTopics() (map[string]*InfoTopic, error) {
	state, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %w", err)
	}

	types, err := n.apiMasterClient.GetTopicTypes()
	if err != nil {
		return nil, fmt.Errorf("getTopicTypes: %w", err)
	}

	ret := make(map[string]*InfoTopic)

	for _, entry := range types {
		ret[entry.Name] = &InfoTopic{
			Type:        entry.Type,
			Publishers:  make(map[string]struct{}),
			Subscribers: make(map[string]struct{}),
		}
	}

	for _, entry := range state.PublishedTopics {
		if _, ok := ret[entry.Name]; !ok {
			continue
		}
		for _, node := range entry.Nodes {
			ret[entry.Name].Publishers[node] = struct{}{}
		}
	}

	for _, entry := range state.SubscribedTopics {
		if _, ok := ret[entry.Name]; !ok {
			continue
		}
		for _, node := range entry.Nodes {
			ret[entry.Name].Subscribers[node] = struct{}{}
		}
	}

	return ret, nil
}

// InfoService contains informations about a service.
type InfoService struct {
	Providers map[string]struct{}
	Address   string
}

// MasterGetServices returns all the services provided by nodes connected to the server.
func (n *Node) MasterGetServices() (map[string]*InfoService, error) {
	state, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %w", err)
	}

	ret := make(map[string]*InfoService)

	for _, entry := range state.ProvidedServices {
		if _, ok := ret[entry.Name]; !ok {
			ret[entry.Name] = &InfoService{
				Providers: make(map[string]struct{}),
			}
		}

		for _, node := range entry.Nodes {
			ret[entry.Name].Providers[node] = struct{}{}
		}

		ur, err := n.apiMasterClient.LookupService(entry.Name)
		if err != nil {
			return nil, fmt.Errorf("lookupService: %w", err)
		}

		address, err := urlToAddress(ur)
		if err != nil {
			return nil, err
		}

		ret[entry.Name].Address = address
	}

	return ret, nil
}
