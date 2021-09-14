// Package apislave implements the Slave API.
//
// https://wiki.ros.org/ROS/Slave_API
package apislave

// Request is a slave API request.
type Request interface{}

// Response is a slave API response.
type Response interface{}

// RequestGetBusInfo is a getBusInfo request.
type RequestGetBusInfo struct {
	CallerID string
}

// ResponseGetBusInfo is the response to a getBusInfo request.
type ResponseGetBusInfo struct {
	Code          int
	StatusMessage string
	BusInfo       [][]interface{}
}

// RequestGetPid is a getPid request.
type RequestGetPid struct {
	CallerID string
}

// ResponseGetPid is the response to a getPid request.
type ResponseGetPid struct {
	Code          int
	StatusMessage string
	Pid           int
}

// RequestGetPublications is a getPublications request.
type RequestGetPublications struct {
	CallerID string
}

// ResponseGetPublications is the response to a getPublications request.
type ResponseGetPublications struct {
	Code          int
	StatusMessage string
	TopicList     [][]string
}

// RequestPublisherUpdate is a publisherUpdate request.
type RequestPublisherUpdate struct {
	CallerID      string
	Topic         string
	PublisherURLs []string
}

// ResponsePublisherUpdate is the response to a publisherUpdate request.
type ResponsePublisherUpdate struct {
	Code          int
	StatusMessage string
	Ignore        int
}

// RequestRequestTopic is a requestTopic request.
type RequestRequestTopic struct {
	CallerID  string
	Topic     string
	Protocols [][]interface{}
}

// ResponseRequestTopic is the response to a requestTopic request.
type ResponseRequestTopic struct {
	Code          int
	StatusMessage string
	Protocol      []interface{}
}

// RequestShutdown is a shutdown request.
type RequestShutdown struct {
	CallerID string
	Reason   string
}

// ResponseShutdown is the response to a shutdown request.
type ResponseShutdown struct {
	Code          int
	StatusMessage string
	Ignore        int
}
