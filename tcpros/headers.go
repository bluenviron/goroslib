package tcpros

type HeaderSubscriber struct {
	Callerid   string
	Md5sum     string
	Topic      string
	Type       string
	TcpNodelay int
}

type HeaderPublisher struct {
	Callerid          string
	Error             string
	Topic             string
	Type              string
	Md5sum            string
	Latching          int
	MessageDefinition string
}

type HeaderServiceClient struct {
	Callerid   string
	Md5sum     string
	Service    string
	Persistent int
}

type HeaderServiceProvider struct {
	Error        string
	Callerid     string
	Md5sum       string
	RequestType  string
	ResponseType string
	Type         string
}
