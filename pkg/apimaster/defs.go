// Package apimaster implements the Master API.
//
// https://wiki.ros.org/ROS/Master_API
package apimaster

type RequestGetPublishedTopics struct {
	CallerID string
	Subgraph string
}

type ResponseGetPublishedTopics struct {
	Code          int
	StatusMessage string
	Topics        [][]string
}

type RequestGetSystemState struct {
	CallerID string
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
	CallerID string
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

type RequestGetURI struct {
	CallerID string
}

type ResponseGetURI struct {
	Code          int
	StatusMessage string
	MasterURI     string
}

type RequestLookup struct {
	CallerID string
	Name     string
}

type ResponseLookup struct {
	Code          int
	StatusMessage string
	URI           string
}

type RequestRegister struct {
	CallerID  string
	Topic     string
	TopicType string
	CallerURL string
}

type ResponseRegister struct {
	Code          int
	StatusMessage string
	URIs          []string
}

type RequestUnregister struct {
	CallerID  string
	Topic     string
	CallerURL string
}

type ResponseUnregister struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}

type RequestRegisterService struct {
	CallerID   string
	Service    string
	ServiceURL string
	CallerURL  string
}

type ResponseServiceRegister struct {
	Code          int
	StatusMessage string
	Ignore        int
}

type RequestUnregisterService struct {
	CallerID   string
	Service    string
	ServiceURL string
}

type ResponseServiceUnregister struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}
