package apiparam

import (
	"fmt"

	"github.com/aler9/goroslib/pkg/xmlrpc"
)

// Client is a Parameter API client.
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

// DeleteParam writes a deleteParam request.
func (c *Client) DeleteParam(req RequestDeleteParam) error {
	req.CallerID = c.callerID

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

// GetParamNames writes a getParamNames request.
func (c *Client) GetParamNames() (*ResponseGetParamNames, error) {
	req := RequestGetParamNames{
		CallerID: c.callerID,
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

// ParamGetBool writes a getParam request and expects a bool response.
func (c *Client) ParamGetBool(key string) (*ResponseParamGetBool, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}

	var res ResponseParamGetBool
	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// ParamGetInt writes a getParam request and expects a int response.
func (c *Client) ParamGetInt(key string) (*ResponseParamGetInt, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}

	var res ResponseParamGetInt
	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// ParamGetString writes a getParam request and expects a string response.
func (c *Client) ParamGetString(key string) (*ResponseParamGetString, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}

	var res ResponseParamGetString
	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res, nil
}

// HasParam writes a hasParam request.
func (c *Client) HasParam(key string) (*ResponseHasParam, error) {
	req := RequestHasParam{
		CallerID: c.callerID,
		Key:      key,
	}

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

// SearchParam writes a searchParam request.
func (c *Client) SearchParam(key string) (*ResponseSearchParam, error) {
	req := RequestSearchParam{
		CallerID: c.callerID,
		Key:      key,
	}

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

// ParamSetBool writes a setParam request.
func (c *Client) ParamSetBool(key string, val bool) error {
	req := RequestParamSetBool{
		CallerID: c.callerID,
		Key:      key,
		Val:      val,
	}

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

// ParamSetInt writes a setParam request.
func (c *Client) ParamSetInt(key string, val int) error {
	req := RequestParamSetInt{
		CallerID: c.callerID,
		Key:      key,
		Val:      val,
	}

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

// ParamSetString writes a setParam request.
func (c *Client) ParamSetString(key string, val string) error {
	req := RequestParamSetString{
		CallerID: c.callerID,
		Key:      key,
		Val:      val,
	}

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
