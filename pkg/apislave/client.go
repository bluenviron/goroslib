package apislave

import (
	"fmt"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

// Client is a Slave API client.
type Client struct {
	xc       *xmlrpc.Client
	callerID string
}

// NewClient allocates a Client.
func NewClient(address string, callerID string) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(address),
		callerID: callerID,
	}
}

// GetPid writes a getPid request.
func (c *Client) GetPid() (int, error) {
	req := RequestGetPid{
		CallerID: c.callerID,
	}
	var res ResponseGetPid

	err := c.xc.Do("getPid", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.Pid, nil
}

// Shutdown writes a shutdown request.
func (c *Client) Shutdown(reason string) error {
	req := RequestShutdown{
		CallerID: c.callerID,
		Reason:   reason,
	}
	var res ResponseShutdown

	err := c.xc.Do("shutdown", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// RequestTopic writes a requestTopic request.
func (c *Client) RequestTopic(topic string, protocols [][]interface{}) (
	[]interface{}, error,
) {
	req := RequestRequestTopic{
		CallerID:  c.callerID,
		Topic:     topic,
		Protocols: protocols,
	}
	var res ResponseRequestTopic

	err := c.xc.Do("requestTopic", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.Protocol, nil
}

// GetBusInfo writes a getBusInfo request.
func (c *Client) GetBusInfo() ([][]interface{}, error) {
	req := RequestGetBusInfo{
		CallerID: c.callerID,
	}
	var res ResponseGetBusInfo

	err := c.xc.Do("getBusInfo", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.BusInfo, nil
}

// GetPublications writes a getPublications request.
func (c *Client) GetPublications() ([][]string, error) {
	req := RequestGetPublications{
		CallerID: c.callerID,
	}
	var res ResponseGetPublications

	err := c.xc.Do("getPublications", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.TopicList, nil
}

// PublisherUpdate writes a publisherUpdate request.
func (c *Client) PublisherUpdate(topic string, urls []string) error {
	req := RequestPublisherUpdate{
		CallerID:      c.callerID,
		Topic:         topic,
		PublisherURLs: urls,
	}
	var res ResponsePublisherUpdate

	err := c.xc.Do("publisherUpdate", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}
