package apiparam

import (
	"fmt"
	"net/http"

	"github.com/bluenviron/goroslib/v2/pkg/xmlrpc"
)

// Client is a Parameter API client.
type Client struct {
	xc       *xmlrpc.Client
	callerID string
}

// NewClient allocates a Client.
func NewClient(address string, callerID string, httpClient *http.Client) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(address, httpClient),
		callerID: callerID,
	}
}

// DeleteParam writes a deleteParam request.
func (c *Client) DeleteParam(key string) error {
	req := RequestDeleteParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseDeleteParam

	err := c.xc.Do("deleteParam", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (code %d)", res.Code)
	}

	return nil
}

// GetParamNames writes a getParamNames request.
func (c *Client) GetParamNames() ([]string, error) {
	req := RequestGetParamNames{
		CallerID: c.callerID,
	}
	var res ResponseGetParamNames

	err := c.xc.Do("getParamNames", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.List, nil
}

// GetParamBool writes a getParam request and expects a bool response.
func (c *Client) GetParamBool(key string) (bool, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseGetParamBool

	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return false, err
	}

	if res.Code != 1 {
		return false, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

// GetParamInt writes a getParam request and expects a int response.
func (c *Client) GetParamInt(key string) (int, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseGetParamInt

	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

// GetParamString writes a getParam request and expects a string response.
func (c *Client) GetParamString(key string) (string, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseGetParamString

	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

// GetParamFloat64 writes a getParam request and expects a float64 response.
func (c *Client) GetParamFloat64(key string) (float64, error) {
	req := RequestGetParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseGetParamFloat64

	err := c.xc.Do("getParam", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return res.Res, nil
}

// HasParam writes a hasParam request.
func (c *Client) HasParam(key string) (bool, error) {
	req := RequestHasParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseHasParam

	err := c.xc.Do("hasParam", req, &res)
	if err != nil {
		return false, err
	}

	if res.Code != 1 {
		return false, fmt.Errorf("server returned an error (code %d)", res.Code)
	}

	if res.KeyOut != req.Key {
		return false, fmt.Errorf("unexpected response")
	}

	return res.Res, nil
}

// SearchParam writes a searchParam request.
func (c *Client) SearchParam(key string) (string, error) {
	req := RequestSearchParam{
		CallerID: c.callerID,
		Key:      key,
	}
	var res ResponseSearchParam

	err := c.xc.Do("searchParam", req, &res)
	if err != nil {
		return "", err
	}

	if res.Code != 1 {
		return "", fmt.Errorf("server returned an error (code %d)", res.Code)
	}

	return res.FoundKey, nil
}

// SetParamBool writes a setParam request.
func (c *Client) SetParamBool(key string, val bool) error {
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
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// SetParamInt writes a setParam request.
func (c *Client) SetParamInt(key string, val int) error {
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
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// SetParamString writes a setParam request.
func (c *Client) SetParamString(key string, val string) error {
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
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

// SetParamFloat64 writes a setParam request.
func (c *Client) SetParamFloat64(key string, val float64) error {
	req := RequestParamSetFloat64{
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
		return fmt.Errorf("server returned an error (code %d): %s", res.Code, res.StatusMessage)
	}

	return nil
}
