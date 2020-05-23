package api_master

import (
	"fmt"

	"github.com/aler9/goroslib/xmlrpc"
)

type Client struct {
	xc       *xmlrpc.Client
	callerId string
}

func NewClient(host string, port uint16, callerId string) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(host, port),
		callerId: callerId,
	}
}

func (c *Client) GetPublishedTopics(req RequestGetPublishedTopics) ([][]string, error) {
	req.CallerId = c.callerId

	var res ResponseGetPublishedTopics
	err := c.xc.Do("getPublishedTopics", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Topics, nil
}

func (c *Client) GetSystemState() (*SystemState, error) {
	req := RequestGetSystemState{
		c.callerId,
	}

	var res ResponseGetSystemState
	err := c.xc.Do("getSystemState", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res.State, nil
}

func (c *Client) GetTopicTypes() ([]TopicType, error) {
	req := RequestGetTopicTypes{
		c.callerId,
	}

	var res ResponseGetTopicTypes
	err := c.xc.Do("getTopicTypes", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Types, nil
}

func (c *Client) GetUri() (string, error) {
	req := RequestGetUri{
		c.callerId,
	}

	var res ResponseGetUri
	err := c.xc.Do("getUri", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.MasterUri, nil
}

func (c *Client) lookup(method string, req RequestLookup) (string, error) {
	req.CallerId = c.callerId

	var res ResponseLookup
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Uri, nil
}

func (c *Client) LookupNode(req RequestLookup) (string, error) {
	return c.lookup("lookupNode", req)
}

func (c *Client) LookupService(req RequestLookup) (string, error) {
	return c.lookup("lookupService", req)
}

func (c *Client) register(method string, req RequestRegister) ([]string, error) {
	req.CallerId = c.callerId

	var res ResponseRegister
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Uris, nil
}

func (c *Client) unregister(method string, req RequestUnregister) error {
	req.CallerId = c.callerId

	var res ResponseUnregister
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return err
	}

	if res.NumUnregistered == 0 {
		return fmt.Errorf("unregister failed")
	}

	return nil
}

func (c *Client) RegisterSubscriber(req RequestRegister) ([]string, error) {
	return c.register("registerSubscriber", req)
}

func (c *Client) UnregisterSubscriber(req RequestUnregister) error {
	return c.unregister("unregisterSubscriber", req)
}

func (c *Client) RegisterPublisher(req RequestRegister) ([]string, error) {
	return c.register("registerPublisher", req)
}

func (c *Client) UnregisterPublisher(req RequestUnregister) error {
	return c.unregister("unregisterPublisher", req)
}

func (c *Client) RegisterService(req RequestRegisterService) error {
	req.CallerId = c.callerId

	var res ResponseServiceRegister
	err := c.xc.Do("registerService", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) UnregisterService(req RequestUnregisterService) error {
	req.CallerId = c.callerId

	var res ResponseServiceUnregister
	err := c.xc.Do("unregisterService", req, &res)
	if err != nil {
		return err
	}

	if res.NumUnregistered == 0 {
		return fmt.Errorf("unregister failed")
	}

	return nil
}
