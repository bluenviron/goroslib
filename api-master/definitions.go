package api_master

type RequestGetSystemState struct {
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

type ResponseGetSystemState struct {
	Code          int
	StatusMessage string
	State         SystemState
}

type RequestGetTopicTypes struct {
	CallerId string
}

type TopicType struct {
	Name string
	Type string
}

type ResponseGetTopicTypes struct {
	Code          int
	StatusMessage string
	Types         []TopicType
}

type RequestLookup struct {
	CallerId string
	Name     string
}

type ResponseLookup struct {
	Code          int
	StatusMessage string
	Uri           string
}

type RequestRegister struct {
	CallerId  string
	Topic     string
	TopicType string
	CallerUrl string
}

type ResponseRegister struct {
	Code          int
	StatusMessage string
	Uris          []string
}

type RequestUnregister struct {
	CallerId  string
	Topic     string
	CallerUrl string
}

type ResponseUnregister struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}

type RequestServiceRegister struct {
	CallerId   string
	Service    string
	ServiceUrl string
	CallerUrl  string
}

type ResponseServiceRegister struct {
	Code          int
	StatusMessage string
	Ignore        int
}

type RequestServiceUnregister struct {
	CallerId   string
	Service    string
	ServiceUrl string
}

type ResponseServiceUnregister struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}
