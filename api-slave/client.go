package api_slave

import (
	"fmt"

	"github.com/aler9/goroslib/xmlrpc"
)

type Client struct {
	xc       *xmlrpc.Client
	callerId string
}

func NewClient(host string, port uint16, callerId string) *Client {
	return &Client{
		xc:       xmlrpc.NewClient(host, port),
		callerId: callerId,
	}
}

func (c *Client) GetPid() (int, error) {
	req := RequestGetPid{
		CallerId: c.callerId,
	}

	var res ResponseGetPid
	err := c.xc.Do("getPid", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Pid, nil
}

func (c *Client) Shutdown(req RequestShutdown) error {
	req.CallerId = c.callerId

	var res ResponseShutdown
	err := c.xc.Do("shutdown", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) RequestTopic(req RequestRequestTopic) (*TopicProtocol, error) {
	req.CallerId = c.callerId

	var res ResponseRequestTopic
	err := c.xc.Do("requestTopic", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res.Proto, nil
}
