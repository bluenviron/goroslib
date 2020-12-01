// Package xmlrpc implements the XML-RPC protocol, in a variant fit for ROS.
package xmlrpc

import (
	"bytes"
	"net/http"
	"net/url"
)

// Client is a XML-RPC client.
type Client struct {
	url string
}

// NewClient allocates a Client.
func NewClient(address string) *Client {
	return &Client{
		url: (&url.URL{
			Scheme: "http",
			Host:   address,
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

	res, err := http.Post(c.url, "text/xml", &buf)
	if err != nil {
		return err
	}
	defer res.Body.Close()

	err = responseDecode(res.Body, paramsRes)
	if err != nil {
		return err
	}

	return nil
}
