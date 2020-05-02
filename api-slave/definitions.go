package api_slave

type Request interface {
	isRequest()
}

type Response interface {
	isResponse()
}

type GetPidReq struct {
	CallerId string
}

func (GetPidReq) isRequest() {}

type GetPidRes struct {
	Code          int
	StatusMessage string
	Pid           int
}

func (GetPidRes) isResponse() {}

type PublisherUpdateReq struct {
	CallerId      string
	Topic         string
	PublisherUrls []string
}

func (PublisherUpdateReq) isRequest() {}

type PublisherUpdateRes struct {
	Code          int
	StatusMessage string
	Ignore        int
}

func (PublisherUpdateRes) isResponse() {}

type RequestTopicReq struct {
	CallerId  string
	Topic     string
	Protocols [][]string
}

func (RequestTopicReq) isRequest() {}

type TopicProtocol struct {
	Name string
	Host string
	Port int
}

type RequestTopicRes struct {
	Code          int
	StatusMessage string
	Proto         TopicProtocol
}

func (RequestTopicRes) isResponse() {}

type ShutdownReq struct {
	CallerId string
	Reason   string
}

func (ShutdownReq) isRequest() {}

type ShutdownRes struct {
	Code          int
	StatusMessage string
	Ignore        int
}

func (ShutdownRes) isResponse() {}
