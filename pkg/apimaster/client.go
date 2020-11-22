package apimaster

import (
	"fmt"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

// Client is a Master API client.
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

// GetPublishedTopics writes a getPublishedTopics request.
func (c *Client) GetPublishedTopics(req RequestGetPublishedTopics) (*ResponseGetPublishedTopics, error) {
	req.CallerId = c.callerId

	var res ResponseGetPublishedTopics
	err := c.xc.Do("getPublishedTopics", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// GetSystemState writes a getSystemState request.
func (c *Client) GetSystemState() (*ResponseGetSystemState, error) {
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

	return &res, nil
}

// GetTopicTypes writes a getTopicTypes request.
func (c *Client) GetTopicTypes() (*ResponseGetTopicTypes, error) {
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

	return &res, nil
}

// GetUri writes a getUri request.
func (c *Client) GetUri() (*ResponseGetUri, error) {
	req := RequestGetUri{
		c.callerId,
	}

	var res ResponseGetUri
	err := c.xc.Do("getUri", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

func (c *Client) lookup(method string, req RequestLookup) (*ResponseLookup, error) {
	req.CallerId = c.callerId

	var res ResponseLookup
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// LookupNode writes a lookupNode request.
func (c *Client) LookupNode(req RequestLookup) (*ResponseLookup, error) {
	return c.lookup("lookupNode", req)
}

// LookupService writes a lookupService request.
func (c *Client) LookupService(req RequestLookup) (*ResponseLookup, error) {
	return c.lookup("lookupService", req)
}

func (c *Client) register(method string, req RequestRegister) (*ResponseRegister, error) {
	req.CallerId = c.callerId

	var res ResponseRegister
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// RegisterSubscriber writes a registerSubscriber request.
func (c *Client) RegisterSubscriber(req RequestRegister) (*ResponseRegister, error) {
	return c.register("registerSubscriber", req)
}

// RegisterPublisher writes a registerPublisher request.
func (c *Client) RegisterPublisher(req RequestRegister) (*ResponseRegister, error) {
	return c.register("registerPublisher", req)
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

// UnregisterSubscriber writes a unregisterSubscriber request.
func (c *Client) UnregisterSubscriber(req RequestUnregister) error {
	return c.unregister("unregisterSubscriber", req)
}

// UnregisterPublisher writes a unregisterPublisher request.
func (c *Client) UnregisterPublisher(req RequestUnregister) error {
	return c.unregister("unregisterPublisher", req)
}

// RegisterService writes a registerService request.
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

// UnregisterService writes a unregisterService request.
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
