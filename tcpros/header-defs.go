package tcpros

type HeaderSubscriber struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
	TcpNodelay        int
}

type HeaderPublisher struct {
	Error    *string
	Topic    *string
	Type     *string
	Md5sum   *string
	Callerid *string
	Latching *int
}

type HeaderServiceClient struct {
	Callerid   string
	Md5sum     string
	Service    string
	Persistent int
}

type HeaderServiceProvider struct {
	Error        *string
	Callerid     *string
	Md5sum       *string
	RequestType  *string
	ResponseType *string
	Type         *string
}
