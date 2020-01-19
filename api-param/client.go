package api_param

import (
	"fmt"

	"github.com/aler9/goroslib/xmlrpc"
)

type Client struct {
	url      string
	callerId string
}

func NewClient(host string, port uint16, callerId string) (*Client, error) {
	return &Client{
		url:      fmt.Sprintf("http://%s:%d/", host, port),
		callerId: callerId,
	}, nil
}

func (c *Client) Close() error {
	return nil
}

func (c *Client) HasParam(name string) (bool, error) {
	req := hasParamReq{
		c.callerId,
		name,
	}
	var res HasParamRes
	err := xmlrpc.Client(c.url, "hasParam", req, &res)
	if err != nil {
		return false, err
	}

	if res.Code != 1 {
		return false, fmt.Errorf("server returned an error (%d)", res.Code)
	}

	if res.NameOut != name {
		return false, fmt.Errorf("unexpected response")
	}

	return res.Res, nil
}

func (c *Client) GetParamBool(name string) (bool, error) {
	req := getParamReq{
		c.callerId,
		name,
	}
	var res getParamBoolRes
	err := xmlrpc.Client(c.url, "getParam", req, &res)
	if err != nil {
		return false, err
	}

	if res.Code != 1 {
		return false, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

func (c *Client) GetParamInt(name string) (int, error) {
	req := getParamReq{
		c.callerId,
		name,
	}
	var res getParamIntRes
	err := xmlrpc.Client(c.url, "getParam", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

func (c *Client) GetParamString(name string) (string, error) {
	req := getParamReq{
		c.callerId,
		name,
	}
	var res getParamStringRes
	err := xmlrpc.Client(c.url, "getParam", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

func (c *Client) SetParamBool(name string, val bool) error {
	req := setParamBoolReq{
		c.callerId,
		name,
		val,
	}
	var res setParamRes
	err := xmlrpc.Client(c.url, "setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) SetParamInt(name string, val int) error {
	req := setParamIntReq{
		c.callerId,
		name,
		val,
	}
	var res setParamRes
	err := xmlrpc.Client(c.url, "setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) SetParamString(name string, val string) error {
	req := setParamStringReq{
		c.callerId,
		name,
		val,
	}
	var res setParamRes
	err := xmlrpc.Client(c.url, "setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// deleteParam

// searchParam

// subscribeParam

// unsubscribeParam

// hasParam

// getParamNames
