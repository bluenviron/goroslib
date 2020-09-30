// protoudp implements the UDPROS protocol.
package protoudp

type HeaderSubscriber struct {
	Callerid string
	Topic    string
	Type     string
	Md5sum   string
}

func (*HeaderSubscriber) IsHeader() {}

type HeaderPublisher struct {
	Callerid          string
	Topic             string
	Type              string
	Md5sum            string
	MessageDefinition string
}

func (*HeaderPublisher) IsHeader() {}
