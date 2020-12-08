// Package protoudp implements the UDPROS protocol.
package protoudp

// HeaderSubscriber is a subscriber header.
type HeaderSubscriber struct {
	Callerid string
	Topic    string
	Type     string
	Md5sum   string
}

// IsHeader implements protocommon.Header.
func (*HeaderSubscriber) IsHeader() {}

// HeaderPublisher is a publisher header.
type HeaderPublisher struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
}

// IsHeader implements protocommon.Header.
func (*HeaderPublisher) IsHeader() {}
