package api_slave

type getPidReq struct {
	CallerId string
}

type ReqGetPid getPidReq

type getPidRes struct {
	Code          int
	StatusMessage string
	Pid           int
}

type publisherUpdateReq struct {
	CallerId      string
	Topic         string
	PublisherUrls []string
}

type ReqPublisherUpdate publisherUpdateReq

type publisherUpdateRes struct {
	Code          int
	StatusMessage string
	Ignore        int
}

type requestTopicReq struct {
	CallerId  string
	Topic     string
	Protocols [][]string
}

type ReqRequestTopic requestTopicReq

type TopicProtocol struct {
	Name string
	Host string
	Port int
}

type requestTopicRes struct {
	Code          int
	StatusMessage string
	Proto         TopicProtocol
}

type shutdownReq struct {
	CallerId string
	Reason   string
}

type ReqShutdown shutdownReq

type shutdownRes struct {
	Code          int
	StatusMessage string
	Ignore        int
}
