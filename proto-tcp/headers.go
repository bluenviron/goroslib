// proto_tcp implements the TCPROS protocol
package proto_tcp

type HeaderError struct {
	Error string
}

func (*HeaderError) IsHeader() {}

type HeaderSubscriber struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
	TcpNodelay        int
}

func (*HeaderSubscriber) IsHeader() {}

type HeaderPublisher struct {
	Topic    string
	Type     string
	Md5sum   string
	Callerid string
	Latching int
}

func (*HeaderPublisher) IsHeader() {}

type HeaderServiceClient struct {
	Callerid   string
	Md5sum     string
	Service    string
	Persistent int
}

func (*HeaderServiceClient) IsHeader() {}

type HeaderServiceProvider struct {
	Callerid     string
	Md5sum       string
	RequestType  string
	ResponseType string
	Type         string
}

func (*HeaderServiceProvider) IsHeader() {}
