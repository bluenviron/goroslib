// Package prototcp implements the TCPROS protocol.
package prototcp

// HeaderError is the header returned in case of errors.
type HeaderError struct {
	Error string
}

// IsHeader implements protocommon.Header.
func (*HeaderError) IsHeader() {}

// HeaderSubscriber is a subscriber header.
type HeaderSubscriber struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
	TcpNodelay        int //nolint:golint
}

// IsHeader implements protocommon.Header.
func (*HeaderSubscriber) IsHeader() {}

// HeaderPublisher is a publisher header.
type HeaderPublisher struct {
	Topic    string
	Type     string
	Md5sum   string
	Callerid string
	Latching int
}

// IsHeader implements protocommon.Header.
func (*HeaderPublisher) IsHeader() {}

// HeaderServiceClient is a service client header.
type HeaderServiceClient struct {
	Callerid   string
	Md5sum     string
	Service    string
	Persistent int
}

// IsHeader implements protocommon.Header.
func (*HeaderServiceClient) IsHeader() {}

// HeaderServiceProvider is a service provider event.
type HeaderServiceProvider struct {
	Callerid     string
	Md5sum       string
	RequestType  string
	ResponseType string
	Type         string
}

// IsHeader implements protocommon.Header.
func (*HeaderServiceProvider) IsHeader() {}
