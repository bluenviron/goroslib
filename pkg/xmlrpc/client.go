// Package xmlrpc implements the XML-RPC protocol, in a variant fit for ROS.
package xmlrpc

import (
	"bytes"
	"fmt"
	"io"
	"net/http"
	"net/url"
)

// Client is a XML-RPC client.
type Client struct {
	httpClient *http.Client
	url        string
}

// NewClient allocates a Client.
func NewClient(host string, httpClient *http.Client) *Client {
	return &Client{
		httpClient: httpClient,
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

	res, err := c.httpClient.Post(c.url, "text/xml", &buf)
	if err != nil {
		return err
	}
	defer res.Body.Close()

	if res.StatusCode != 200 {
		io.Copy(io.Discard, res.Body)
		return fmt.Errorf("bad status code: %d", res.StatusCode)
	}

	err = responseDecode(res.Body, paramsRes)
	if err != nil {
		return err
	}

	return nil
}
