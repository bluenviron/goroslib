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

func (c *Client) DeleteParam(req RequestDeleteParam) error {
	req.CallerId = c.callerId

	var res ResponseDeleteParam
	err := xmlrpc.Client(c.url, "deleteParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d)", res.Code)
	}

	return nil
}

func (c *Client) GetParamNames() ([]string, error) {
	req := RequestGetParamNames{
		CallerId: c.callerId,
	}

	var res ResponseGetParamNames
	err := xmlrpc.Client(c.url, "getParamNames", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.List, nil
}

func (c *Client) GetParamBool(req RequestGetParam) (bool, error) {
	req.CallerId = c.callerId

	var res ResponseGetParamBool
	err := xmlrpc.Client(c.url, "getParam", req, &res)
	if err != nil {
		return false, err
	}

	if res.Code != 1 {
		return false, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

func (c *Client) GetParamInt(req RequestGetParam) (int, error) {
	req.CallerId = c.callerId

	var res ResponseGetParamInt
	err := xmlrpc.Client(c.url, "getParam", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

func (c *Client) GetParamString(req RequestGetParam) (string, error) {
	req.CallerId = c.callerId

	var res ResponseGetParamString
	err := xmlrpc.Client(c.url, "getParam", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

func (c *Client) HasParam(req RequestHasParam) (bool, error) {
	req.CallerId = c.callerId

	var res ResponseHasParam
	err := xmlrpc.Client(c.url, "hasParam", req, &res)
	if err != nil {
		return false, err
	}

	if res.Code != 1 {
		return false, fmt.Errorf("server returned an error (%d)", res.Code)
	}

	if res.KeyOut != req.Key {
		return false, fmt.Errorf("unexpected response")
	}

	return res.Res, nil
}

func (c *Client) SearchParam(req RequestSearchParam) (string, error) {
	req.CallerId = c.callerId

	var res ResponseSearchParam
	err := xmlrpc.Client(c.url, "searchParam", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (%d)", res.Code)
	}

	return res.FoundKey, nil
}

func (c *Client) SetParamBool(req RequestSetParamBool) error {
	req.CallerId = c.callerId

	var res ResponseSetParam
	err := xmlrpc.Client(c.url, "setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) SetParamInt(req RequestSetParamInt) error {
	req.CallerId = c.callerId

	var res ResponseSetParam
	err := xmlrpc.Client(c.url, "setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) SetParamString(req RequestSetParamString) error {
	req.CallerId = c.callerId

	var res ResponseSetParam
	err := xmlrpc.Client(c.url, "setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}
