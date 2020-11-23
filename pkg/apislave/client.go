package apislave

import (
	"fmt"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

// Client is a Slave API client.
type Client struct {
	xc       *xmlrpc.Client
	callerId string
}

// NewClient allocates a Client.
func NewClient(host string, port int, callerId string) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(host, port),
		callerId: callerId,
	}
}

// GetPid writes a getPid request.
func (c *Client) GetPid() (*ResponseGetPid, error) {
	req := RequestGetPid{
		CallerId: c.callerId,
	}

	var res ResponseGetPid
	err := c.xc.Do("getPid", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// Shutdown writes a shutdown request.
func (c *Client) Shutdown(reason string) error {
	req := RequestShutdown{
		CallerId: c.callerId,
		Reason:   reason,
	}

	var res ResponseShutdown
	err := c.xc.Do("shutdown", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// RequestTopic writes a requestTopic request.
func (c *Client) RequestTopic(topic string, protocols [][]interface{}) (*ResponseRequestTopic, error) {
	req := RequestRequestTopic{
		CallerId:  c.callerId,
		Topic:     topic,
		Protocols: protocols,
	}

	var res ResponseRequestTopic
	err := c.xc.Do("requestTopic", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}
