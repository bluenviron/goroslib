package api_slave

import (
	"fmt"

	"github.com/aler9/goroslib/xmlrpc"
)

type Client struct {
	api      string
	callerId string
}

func NewClient(host string, port uint16, callerId string) (*Client, error) {
	return &Client{
		api:      fmt.Sprintf("http://%s:%d/", host, port),
		callerId: callerId,
	}, nil
}

func (c *Client) Close() error {
	return nil
}

func (c *Client) GetPid() (int, error) {
	req := getPidReq{
		CallerId: c.callerId,
	}
	var res getPidRes
	err := xmlrpc.Client(c.api, "getPid", req, &res)
	if err != nil {
		return 0, err
	}

	if res.Code != 1 {
		return 0, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return res.Pid, nil
}

func (c *Client) Shutdown(reason string) error {
	req := shutdownReq{
		CallerId: c.callerId,
		Reason:   reason,
	}
	var res shutdownRes
	err := xmlrpc.Client(c.api, "shutdown", req, &res)
	if err != nil {
		return err
	}

	if res.Code != 1 {
		return fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return nil
}

func (c *Client) RequestTopic(topic string, protocols [][]string) (*TopicProtocol, error) {
	req := requestTopicReq{
		CallerId:  c.callerId,
		Topic:     topic,
		Protocols: protocols,
	}
	var res requestTopicRes
	err := xmlrpc.Client(c.api, "requestTopic", req, &res)
	if err != nil {
		return nil, err
	}

	if res.Code != 1 {
		return nil, fmt.Errorf("server returned an error (%d): %s", res.Code, res.StatusMessage)
	}

	return &res.Proto, nil
}
