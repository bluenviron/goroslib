// Package apislave implements the Slave API.
//
// https://wiki.ros.org/ROS/Slave_API
package apislave

// Request is a slave API request.
type Request interface {
	isRequest()
}

// Response is a slave API response.
type Response interface {
	isResponse()
}

// RequestGetBusInfo is a getBusInfo request.
type RequestGetBusInfo struct {
	CallerID string
}

func (RequestGetBusInfo) isRequest() {}

// ResponseGetBusInfo is the response to a getBusInfo request.
type ResponseGetBusInfo struct {
	Code          int
	StatusMessage string
	BusInfo       [][]interface{}
}

func (ResponseGetBusInfo) isResponse() {}

// RequestGetPid is a getPid request.
type RequestGetPid struct {
	CallerID string
}

func (RequestGetPid) isRequest() {}

// ResponseGetPid is the response to a getPid request.
type ResponseGetPid struct {
	Code          int
	StatusMessage string
	Pid           int
}

func (ResponseGetPid) isResponse() {}

// RequestGetPublications is a getPublications request.
type RequestGetPublications struct {
	CallerID string
}

func (RequestGetPublications) isRequest() {}

// ResponseGetPublications is the response to a getPublications request.
type ResponseGetPublications struct {
	Code          int
	StatusMessage string
	TopicList     [][]string
}

func (ResponseGetPublications) isResponse() {}

// RequestPublisherUpdate is a publisherUpdate request.
type RequestPublisherUpdate struct {
	CallerID      string
	Topic         string
	PublisherURLs []string
}

func (RequestPublisherUpdate) isRequest() {}

// ResponsePublisherUpdate is the response to a publisherUpdate request.
type ResponsePublisherUpdate struct {
	Code          int
	StatusMessage string
	Ignore        int
}

func (ResponsePublisherUpdate) isResponse() {}

// RequestRequestTopic is a requestTopic request.
type RequestRequestTopic struct {
	CallerID  string
	Topic     string
	Protocols [][]interface{}
}

func (RequestRequestTopic) isRequest() {}

// ResponseRequestTopic is the response to a requestTopic request.
type ResponseRequestTopic struct {
	Code          int
	StatusMessage string
	Protocol      []interface{}
}

func (ResponseRequestTopic) isResponse() {}

// RequestShutdown is a shutdown request.
type RequestShutdown struct {
	CallerID string
	Reason   string
}

func (RequestShutdown) isRequest() {}

// ResponseShutdown is the response to a shutdown request.
type ResponseShutdown struct {
	Code          int
	StatusMessage string
	Ignore        int
}

func (ResponseShutdown) isResponse() {}
