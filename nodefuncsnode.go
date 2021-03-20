package goroslib

import (
	"fmt"
	"time"

	"github.com/aler9/goroslib/pkg/apislave"
)

// InfoConnection contains information about a connection.
type InfoConnection struct {
	ID        int
	To        string
	Direction byte
	Transport string
	Topic     string
	Connected bool
}

// NodeGetConns returns infos about connections of a node.
func (n *Node) NodeGetConns(nodeName string) ([]InfoConnection, error) {
	res, err := n.apiMasterClient.LookupNode(nodeName)
	if err != nil {
		return nil, err
	}

	address, err := urlToAddress(res.URL)
	if err != nil {
		return nil, err
	}

	xcs := apislave.NewClient(address, n.absoluteName())

	infos, err := xcs.GetBusInfo()
	if err != nil {
		return nil, err
	}

	ret := make([]InfoConnection, len(infos))

	for i, info := range infos {
		if len(info) < 6 {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}

		id, ok := info[0].(int)
		if !ok {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}

		counterpart, ok := info[1].(string)
		if !ok {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}

		temp, ok := info[2].(string)
		if !ok || len(temp) != 1 {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}
		direction := temp[0]

		transport, ok := info[3].(string)
		if !ok {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}

		topic, ok := info[4].(string)
		if !ok {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}

		connected, ok := info[5].(bool)
		if !ok {
			return nil, fmt.Errorf("invalid entry: %v", info)
		}

		ret[i] = InfoConnection{
			ID:        id,
			To:        counterpart,
			Direction: direction,
			Transport: transport,
			Topic:     topic,
			Connected: connected,
		}
	}

	return ret, nil
}

// NodePing sends a ping request to a given node, wait for the response and returns
// the elapsed time.
func (n *Node) NodePing(nodeName string) (time.Duration, error) {
	res, err := n.apiMasterClient.LookupNode(nodeName)
	if err != nil {
		return 0, err
	}

	address, err := urlToAddress(res.URL)
	if err != nil {
		return 0, err
	}

	xcs := apislave.NewClient(address, n.absoluteName())

	start := time.Now()

	_, err = xcs.GetPid()
	if err != nil {
		return 0, err
	}

	return time.Since(start), nil
}

// NodeKill sends a kill request to a given node.
func (n *Node) NodeKill(nodeName string) error {
	res, err := n.apiMasterClient.LookupNode(nodeName)
	if err != nil {
		return err
	}

	address, err := urlToAddress(res.URL)
	if err != nil {
		return err
	}

	xcs := apislave.NewClient(address, n.absoluteName())

	err = xcs.Shutdown("")
	if err != nil {
		return err
	}

	return nil
}
