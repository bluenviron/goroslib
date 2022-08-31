// Package xmlrpc implements the XML-RPC protocol, in a variant fit for ROS.
package xmlrpc

import (
	"bytes"
	"fmt"
	"net/http"
	"net/url"
	"time"
)

const (
	clientTimeout = 10 * time.Second
)

var httpClient = &http.Client{
	Timeout: clientTimeout,
}

// Client is a XML-RPC client.
type Client struct {
	url string
}

// NewClient allocates a Client.
func NewClient(host string) *Client {
	return &Client{
		url: (&url.URL{
			Scheme: "http",
			Host:   host,
			Path:   "/RPC2",
		}).String(),
	}
}

// Do writes a request and reads a response.
func (c *Client) Do(method string, paramsReq interface{}, paramsRes interface{}) error {
	var buf bytes.Buffer
	err := requestEncode(&buf, method, paramsReq)
	if err != nil {
		return err
	}

	res, err := httpClient.Post(c.url, "text/xml", &buf)
	if err != nil {
		return err
	}
	defer res.Body.Close()

	if res.StatusCode != 200 {
		return fmt.Errorf("bad status code: %d", res.StatusCode)
	}

	err = responseDecode(res.Body, paramsRes)
	if err != nil {
		return err
	}

	return nil
}
