package apiparam

import (
	"fmt"

	"github.com/aler9/goroslib/xmlrpc"
)

type Client struct {
	xc       *xmlrpc.Client
	callerId string
}

func NewClient(host string, port int, callerId string) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(host, port),
		callerId: callerId,
	}
}

func (c *Client) DeleteParam(req RequestDeleteParam) error {
	req.CallerId = c.callerId

	var res ResponseDeleteParam
	err := c.xc.Do("deleteParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d)", res.Code)
	}

	return nil
}

func (c *Client) GetParamNames() (*ResponseGetParamNames, error) {
	req := RequestGetParamNames{
		CallerId: c.callerId,
	}

	var res ResponseGetParamNames
	err := c.xc.Do("getParamNames", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

func (c *Client) GetParamBool(req RequestGetParam) (*ResponseGetParamBool, error) {
	req.CallerId = c.callerId

	var res ResponseGetParamBool
	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

func (c *Client) GetParamInt(req RequestGetParam) (*ResponseGetParamInt, error) {
	req.CallerId = c.callerId

	var res ResponseGetParamInt
	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

func (c *Client) GetParamString(req RequestGetParam) (*ResponseGetParamString, error) {
	req.CallerId = c.callerId

	var res ResponseGetParamString
	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

func (c *Client) HasParam(req RequestHasParam) (*ResponseHasParam, error) {
	req.CallerId = c.callerId

	var res ResponseHasParam
	err := c.xc.Do("hasParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d)", res.Code)
	}

	if res.KeyOut != req.Key {
		return nil, fmt.Errorf("unexpected response")
	}

	return &res, nil
}

func (c *Client) SearchParam(req RequestSearchParam) (*ResponseSearchParam, error) {
	req.CallerId = c.callerId

	var res ResponseSearchParam
	err := c.xc.Do("searchParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d)", res.Code)
	}

	return &res, nil
}

func (c *Client) SetParamBool(req RequestSetParamBool) error {
	req.CallerId = c.callerId

	var res ResponseSetParam
	err := c.xc.Do("setParam", req, &res)
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
	err := c.xc.Do("setParam", req, &res)
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
	err := c.xc.Do("setParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}
