// api_slave implements the Slave API, documented here: https://wiki.ros.org/ROS/Slave_API
package api_slave

type Request interface {
	isRequest()
}

type Response interface {
	isResponse()
}

type RequestGetBusInfo struct {
	CallerId string
}

func (RequestGetBusInfo) isRequest() {}

type ResponseGetBusInfo struct {
	Code          int
	StatusMessage string
	BusInfo       [][]string
}

func (ResponseGetBusInfo) isResponse() {}

type RequestGetPid struct {
	CallerId string
}

func (RequestGetPid) isRequest() {}

type ResponseGetPid struct {
	Code          int
	StatusMessage string
	Pid           int
}

func (ResponseGetPid) isResponse() {}

type RequestGetPublications struct {
	CallerId string
}

func (RequestGetPublications) isRequest() {}

type ResponseGetPublications struct {
	Code          int
	StatusMessage string
	TopicList     [][]string
}

func (ResponseGetPublications) isResponse() {}

type RequestPublisherUpdate struct {
	CallerId      string
	Topic         string
	PublisherUrls []string
}

func (RequestPublisherUpdate) isRequest() {}

type ResponsePublisherUpdate struct {
	Code          int
	StatusMessage string
	Ignore        int
}

func (ResponsePublisherUpdate) isResponse() {}

type RequestRequestTopic struct {
	CallerId  string
	Topic     string
	Protocols [][]string
}

func (RequestRequestTopic) isRequest() {}

type TopicProtocol struct {
	Name string
	Host string
	Port int
}

type ResponseRequestTopic struct {
	Code          int
	StatusMessage string
	Proto         TopicProtocol
}

func (ResponseRequestTopic) isResponse() {}

type RequestShutdown struct {
	CallerId string
	Reason   string
}

func (RequestShutdown) isRequest() {}

type ResponseShutdown struct {
	Code          int
	StatusMessage string
	Ignore        int
}

func (ResponseShutdown) isResponse() {}
