// Package prototcp implements the TCPROS protocol.
package prototcp

// HeaderError is the header returned in case of errors.
type HeaderError struct {
	Error string
}

// HeaderSubscriber is a subscriber header.
type HeaderSubscriber struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
	TcpNodelay        int //nolint:revive
}

// HeaderPublisher is a publisher header.
type HeaderPublisher struct {
	Topic             string
	Type              string
	Md5sum            string
	Callerid          string
	Latching          int
	MessageDefinition string
}

// HeaderServiceClient is a service client header.
type HeaderServiceClient struct {
	Callerid   string
	Md5sum     string
	Service    string
	Persistent int
}

// HeaderServiceProvider is a service provider event.
type HeaderServiceProvider struct {
	Callerid     string
	Md5sum       string
	RequestType  string
	ResponseType string
	Type         string
}
