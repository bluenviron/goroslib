// proto_udp implements the UDPROS protocol
package proto_udp

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
