package goroslib

import (
	"fmt"
	"time"

	"github.com/aler9/goroslib/pkg/apislave"
)

// InfoNode contains informations about a node.
type InfoNode struct {
	PublishedTopics  map[string]struct{}
	SubscribedTopics map[string]struct{}
	ProvidedServices map[string]struct{}
	Hostname         string
	Port             int
}

// GetNodes returns all the nodes connected to the master.
func (n *Node) GetNodes() (map[string]*InfoNode, error) {
	res, err := n.apiMasterClient.GetSystemState()
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

	for _, entry := range res.State.PublishedTopics {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].PublishedTopics[entry.Name] = struct{}{}
		}
	}

	for _, entry := range res.State.SubscribedTopics {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].SubscribedTopics[entry.Name] = struct{}{}
		}
	}

	for _, entry := range res.State.ProvidedServices {
		for _, node := range entry.Nodes {
			initEntry(node)
			ret[node].ProvidedServices[entry.Name] = struct{}{}
		}
	}

	for node, info := range ret {
		res, err := n.apiMasterClient.LookupNode(node)
		if err != nil {
			return nil, fmt.Errorf("lookupNode: %v", err)
		}

		hostname, port, err := parseUrl(res.Uri)
		if err != nil {
			return nil, err
		}

		info.Hostname = hostname
		info.Port = port
	}

	return ret, nil
}

// GetMachines returns all the machines connected to the master through a node.
func (n *Node) GetMachines() (map[string]struct{}, error) {
	// this is like its equivalent in python
	// https://docs.ros.org/melodic/api/rosnode/html/rosnode-pysrc.html#get_machines_by_nodes

	nodes, err := n.GetNodes()
	if err != nil {
		return nil, err
	}

	ret := make(map[string]struct{})
	for _, info := range nodes {
		ret[info.Hostname] = struct{}{}
	}

	return ret, nil
}

// InfoTopic contains informations about a topic.
type InfoTopic struct {
	Type        string
	Publishers  map[string]struct{}
	Subscribers map[string]struct{}
}

// GetTopics returns all the topics published by nodes connected to the master.
func (n *Node) GetTopics() (map[string]*InfoTopic, error) {
	res1, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %v", err)
	}

	res2, err := n.apiMasterClient.GetTopicTypes()
	if err != nil {
		return nil, fmt.Errorf("getTopicTypes: %v", err)
	}

	ret := make(map[string]*InfoTopic)

	for _, entry := range res2.Types {
		ret[entry.Name] = &InfoTopic{
			Type:        entry.Type,
			Publishers:  make(map[string]struct{}),
			Subscribers: make(map[string]struct{}),
		}
	}

	for _, entry := range res1.State.PublishedTopics {
		if _, ok := ret[entry.Name]; !ok {
			continue
		}
		for _, node := range entry.Nodes {
			ret[entry.Name].Publishers[node] = struct{}{}
		}
	}

	for _, entry := range res1.State.SubscribedTopics {
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
	Hostname  string
	Port      int
}

// GetServices returns all the services provided by nodes connected to the server.
func (n *Node) GetServices() (map[string]*InfoService, error) {
	res1, err := n.apiMasterClient.GetSystemState()
	if err != nil {
		return nil, fmt.Errorf("getSystemState: %v", err)
	}

	ret := make(map[string]*InfoService)

	for _, entry := range res1.State.ProvidedServices {
		if _, ok := ret[entry.Name]; !ok {
			ret[entry.Name] = &InfoService{
				Providers: make(map[string]struct{}),
			}
		}

		for _, node := range entry.Nodes {
			ret[entry.Name].Providers[node] = struct{}{}
		}

		res2, err := n.apiMasterClient.LookupService(entry.Name)
		if err != nil {
			return nil, fmt.Errorf("lookupService: %v", err)
		}

		hostname, port, err := parseUrl(res2.Uri)
		if err != nil {
			return nil, err
		}

		ret[entry.Name].Hostname = hostname
		ret[entry.Name].Port = port
	}

	return ret, nil
}

// PingNode send a ping request to a given node, wait for the response and returns
// the elapsed time.
func (n *Node) PingNode(name string) (time.Duration, error) {
	res, err := n.apiMasterClient.LookupNode(name)
	if err != nil {
		return 0, err
	}

	hostname, port, err := parseUrl(res.Uri)
	if err != nil {
		return 0, err
	}

	xcs := apislave.NewClient(hostname, port, n.conf.Name)

	start := time.Now()

	_, err = xcs.GetPid()
	if err != nil {
		return 0, err
	}

	return time.Since(start), nil
}

// KillNode send a kill request to a given node.
func (n *Node) KillNode(name string) error {
	res, err := n.apiMasterClient.LookupNode(name)
	if err != nil {
		return err
	}

	hostname, port, err := parseUrl(res.Uri)
	if err != nil {
		return err
	}

	xcs := apislave.NewClient(hostname, port, n.conf.Name)

	err = xcs.Shutdown("")
	if err != nil {
		return err
	}

	return nil
}

// GetParamBool returns a bool parameter from the master.
func (n *Node) GetParamBool(key string) (bool, error) {
	res, err := n.apiParamClient.GetParamBool(key)
	if err != nil {
		return false, err
	}
	return res.Res, nil
}

// GetParamInt returns an int parameter from the master.
func (n *Node) GetParamInt(key string) (int, error) {
	res, err := n.apiParamClient.GetParamInt(key)
	if err != nil {
		return 0, err
	}
	return res.Res, nil
}

// GetParamString returns a string parameter from the master.
func (n *Node) GetParamString(key string) (string, error) {
	res, err := n.apiParamClient.GetParamString(key)
	if err != nil {
		return "", err
	}
	return res.Res, nil
}

// SetParamBool sets a bool parameter in the master.
func (n *Node) SetParamBool(key string, val bool) error {
	return n.apiParamClient.SetParamBool(key, val)
}

// SetParamInt sets an int parameter in the master.
func (n *Node) SetParamInt(key string, val int) error {
	return n.apiParamClient.SetParamInt(key, val)
}

// SetParamString sets a string parameter in the master.
func (n *Node) SetParamString(key string, val string) error {
	return n.apiParamClient.SetParamString(key, val)
}
