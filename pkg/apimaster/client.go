package apimaster

import (
	"fmt"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

// Client is a Master API client.
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

// GetPublishedTopics writes a getPublishedTopics request.
func (c *Client) GetPublishedTopics(subgraph string) ([][]string, error) {
	req := RequestGetPublishedTopics{
		CallerID: c.callerID,
		Subgraph: subgraph,
	}
	var res ResponseGetPublishedTopics

	err := c.xc.Do("getPublishedTopics", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	return res.Topics, nil
}

// GetSystemState writes a getSystemState request.
func (c *Client) GetSystemState() (*SystemState, error) {
	req := RequestGetSystemState{
		c.callerID,
	}
	var res ResponseGetSystemState

	err := c.xc.Do("getSystemState", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	return &res.State, nil
}

// GetTopicTypes writes a getTopicTypes request.
func (c *Client) GetTopicTypes() ([]TopicType, error) {
	req := RequestGetTopicTypes{
		c.callerID,
	}
	var res ResponseGetTopicTypes

	err := c.xc.Do("getTopicTypes", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	return res.Types, nil
}

// GetURI writes a getUri request.
func (c *Client) GetURI() (string, error) {
	req := RequestGetURI{
		c.callerID,
	}
	var res ResponseGetURI

	err := c.xc.Do("getUri", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	return res.MasterURI, nil
}

func (c *Client) lookup(method string, name string) (string, error) {
	req := RequestLookup{
		CallerID: c.callerID,
		Name:     name,
	}
	var res ResponseLookup

	err := c.xc.Do(method, req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	return res.URL, nil
}

// LookupNode writes a lookupNode request.
func (c *Client) LookupNode(name string) (string, error) {
	return c.lookup("lookupNode", name)
}

// LookupService writes a lookupService request.
func (c *Client) LookupService(name string) (string, error) {
	return c.lookup("lookupService", name)
}

func (c *Client) register(method string, topic string, topicType string,
	callerURL string,
) ([]string, error) {
	req := RequestRegister{
		CallerID:  c.callerID,
		Topic:     topic,
		TopicType: topicType,
		CallerURL: callerURL,
	}
	var res ResponseRegister

	err := c.xc.Do(method, req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	return res.URIs, nil
}

// RegisterSubscriber writes a registerSubscriber request.
func (c *Client) RegisterSubscriber(
	topic string,
	topicType string,
	callerURL string,
) ([]string, error) {
	return c.register("registerSubscriber", topic, topicType, callerURL)
}

// RegisterPublisher writes a registerPublisher request.
func (c *Client) RegisterPublisher(
	topic string,
	topicType string,
	callerURL string,
) ([]string, error) {
	return c.register("registerPublisher", topic, topicType, callerURL)
}

func (c *Client) unregister(method string, topic string, callerURL string) error {
	req := RequestUnregister{
		CallerID:  c.callerID,
		Topic:     topic,
		CallerURL: callerURL,
	}
	var res ResponseUnregister

	err := c.xc.Do(method, req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	if res.NumUnregistered == 0 {
		return fmt.Errorf("unregister failed")
	}

	return nil
}

// UnregisterSubscriber writes a unregisterSubscriber request.
func (c *Client) UnregisterSubscriber(topic string, callerURL string) error {
	return c.unregister("unregisterSubscriber", topic, callerURL)
}

// UnregisterPublisher writes a unregisterPublisher request.
func (c *Client) UnregisterPublisher(topic string, callerURL string) error {
	return c.unregister("unregisterPublisher", topic, callerURL)
}

// RegisterService writes a registerService request.
func (c *Client) RegisterService(
	service string,
	serviceURL string,
	callerURL string,
) error {
	req := RequestRegisterService{
		CallerID:   c.callerID,
		Service:    service,
		ServiceURL: serviceURL,
		CallerURL:  callerURL,
	}
	var res ResponseRegisterService

	err := c.xc.Do("registerService", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// UnregisterService writes a unregisterService request.
func (c *Client) UnregisterService(
	service string,
	serviceURL string,
) error {
	req := RequestUnregisterService{
		CallerID:   c.callerID,
		Service:    service,
		ServiceURL: serviceURL,
	}
	var res ResponseServiceUnregister

	err := c.xc.Do("unregisterService", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (code %d): %s", res.Code,
			res.StatusMessage)
	}

	if res.NumUnregistered == 0 {
		return fmt.Errorf("unregister failed")
	}

	return nil
}
