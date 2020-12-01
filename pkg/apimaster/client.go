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
func NewClient(address string, callerId string) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(address),
		callerId: callerId,
	}
}

// GetPublishedTopics writes a getPublishedTopics request.
func (c *Client) GetPublishedTopics(subgraph string) (*ResponseGetPublishedTopics, error) {
	req := RequestGetPublishedTopics{
		CallerId: c.callerId,
		Subgraph: subgraph,
	}

	var res ResponseGetPublishedTopics
	err := c.xc.Do("getPublishedTopics", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code,
			res.StatusMessage)
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
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code,
			res.StatusMessage)
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
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code,
			res.StatusMessage)
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
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code,
			res.StatusMessage)
	}

	return &res, nil
}

func (c *Client) lookup(method string, name string) (*ResponseLookup, error) {
	req := RequestLookup{
		CallerId: c.callerId,
		Name:     name,
	}

	var res ResponseLookup
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code,
			res.StatusMessage)
	}

	return &res, nil
}

// LookupNode writes a lookupNode request.
func (c *Client) LookupNode(name string) (*ResponseLookup, error) {
	return c.lookup("lookupNode", name)
}

// LookupService writes a lookupService request.
func (c *Client) LookupService(name string) (*ResponseLookup, error) {
	return c.lookup("lookupService", name)
}

func (c *Client) register(method string, topic string, topicType string,
	callerUrl string) (*ResponseRegister, error) {
	req := RequestRegister{
		CallerId:  c.callerId,
		Topic:     topic,
		TopicType: topicType,
		CallerUrl: callerUrl,
	}

	var res ResponseRegister
	err := c.xc.Do(method, req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code,
			res.StatusMessage)
	}

	return &res, nil
}

// RegisterSubscriber writes a registerSubscriber request.
func (c *Client) RegisterSubscriber(topic string, topicType string,
	callerUrl string) (*ResponseRegister, error) {
	return c.register("registerSubscriber", topic, topicType, callerUrl)
}

// RegisterPublisher writes a registerPublisher request.
func (c *Client) RegisterPublisher(topic string, topicType string,
	callerUrl string) (*ResponseRegister, error) {
	return c.register("registerPublisher", topic, topicType, callerUrl)
}

func (c *Client) unregister(method string, topic string, callerUrl string) error {
	req := RequestUnregister{
		CallerId:  c.callerId,
		Topic:     topic,
		CallerUrl: callerUrl,
	}

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
func (c *Client) UnregisterSubscriber(topic string, callerUrl string) error {
	return c.unregister("unregisterSubscriber", topic, callerUrl)
}

// UnregisterPublisher writes a unregisterPublisher request.
func (c *Client) UnregisterPublisher(topic string, callerUrl string) error {
	return c.unregister("unregisterPublisher", topic, callerUrl)
}

// RegisterService writes a registerService request.
func (c *Client) RegisterService(service string, serviceUrl string,
	callerUrl string) error {
	req := RequestRegisterService{
		CallerId:   c.callerId,
		Service:    service,
		ServiceUrl: serviceUrl,
		CallerUrl:  callerUrl,
	}

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
func (c *Client) UnregisterService(service string, serviceUrl string) error {
	req := RequestUnregisterService{
		CallerId:   c.callerId,
		Service:    service,
		ServiceUrl: serviceUrl,
	}

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
