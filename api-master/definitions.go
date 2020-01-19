package api_master

type getSystemStateReq struct {
	CallerId string
}

type SystemStateEntry struct {
	Name  string
	Nodes []string
}

type SystemState struct {
	PublishedTopics  []SystemStateEntry
	SubscribedTopics []SystemStateEntry
	ProvidedServices []SystemStateEntry
}

type getSystemStateRes struct {
	Code          int
	StatusMessage string
	State         SystemState
}

type getTopicTypesReq struct {
	CallerId string
}

type TopicType struct {
	Name string
	Type string
}

type getTopicTypesRes struct {
	Code          int
	StatusMessage string
	Types         []TopicType
}

type lookupReq struct {
	CallerId string
	Name     string
}

type lookupRes struct {
	Code          int
	StatusMessage string
	Uri           string
}

type registerReq struct {
	CallerId  string
	Topic     string
	TopicType string
	CallerUrl string
}

type registerRes struct {
	Code          int
	StatusMessage string
	Uris          []string
}

type unregisterReq struct {
	CallerId  string
	Topic     string
	CallerUrl string
}

type unregisterRes struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}

type serviceRegisterReq struct {
	CallerId   string
	Service    string
	ServiceUrl string
	CallerUrl  string
}

type serviceRegisterRes struct {
	Code          int
	StatusMessage string
	Ignore        int
}

type serviceUnregisterReq struct {
	CallerId   string
	Service    string
	ServiceUrl string
}

type serviceUnregisterRes struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}
