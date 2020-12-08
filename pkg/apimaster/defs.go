// Package apimaster implements the Master API.
//
// https://wiki.ros.org/ROS/Master_API
package apimaster

// RequestGetPublishedTopics is a getPublishedTopics request.
type RequestGetPublishedTopics struct {
	CallerID string
	Subgraph string
}

// ResponseGetPublishedTopics is the response to a getPublishedTopics request.
type ResponseGetPublishedTopics struct {
	Code          int
	StatusMessage string
	Topics        [][]string
}

// RequestGetSystemState is a getSystemState request.
type RequestGetSystemState struct {
	CallerID string
}

// SystemStateEntry is a system state entry.
type SystemStateEntry struct {
	Name  string
	Nodes []string
}

// SystemState is a system state.
type SystemState struct {
	PublishedTopics  []SystemStateEntry
	SubscribedTopics []SystemStateEntry
	ProvidedServices []SystemStateEntry
}

// ResponseGetSystemState is the response to a getSystemState request.
type ResponseGetSystemState struct {
	Code          int
	StatusMessage string
	State         SystemState
}

// RequestGetTopicTypes is a getTopicTypes request.
type RequestGetTopicTypes struct {
	CallerID string
}

// TopicType is a topic type.
type TopicType struct {
	Name string
	Type string
}

// ResponseGetTopicTypes is the response to a getTopicTypes request.
type ResponseGetTopicTypes struct {
	Code          int
	StatusMessage string
	Types         []TopicType
}

// RequestGetURI is a getUri request.
type RequestGetURI struct {
	CallerID string
}

// ResponseGetURI is the response to a getUri request.
type ResponseGetURI struct {
	Code          int
	StatusMessage string
	MasterURI     string
}

// RequestLookup is a lookup* request.
type RequestLookup struct {
	CallerID string
	Name     string
}

// ResponseLookup is the response to a lookup* request.
type ResponseLookup struct {
	Code          int
	StatusMessage string
	URL           string
}

// RequestRegister is a register* request.
type RequestRegister struct {
	CallerID  string
	Topic     string
	TopicType string
	CallerURL string
}

// ResponseRegister is the response to a register* request.
type ResponseRegister struct {
	Code          int
	StatusMessage string
	URIs          []string
}

// RequestUnregister is an unregister* request.
type RequestUnregister struct {
	CallerID  string
	Topic     string
	CallerURL string
}

// ResponseUnregister is the response to a unregister* request.
type ResponseUnregister struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}

// RequestRegisterService is a registerService request.
type RequestRegisterService struct {
	CallerID   string
	Service    string
	ServiceURL string
	CallerURL  string
}

// ResponseRegisterService is the response to a registerService request.
type ResponseRegisterService struct {
	Code          int
	StatusMessage string
	Ignore        int
}

// RequestUnregisterService is a unregisterService request.
type RequestUnregisterService struct {
	CallerID   string
	Service    string
	ServiceURL string
}

// ResponseServiceUnregister is the response to a unregisterService request.
type ResponseServiceUnregister struct {
	Code            int
	StatusMessage   string
	NumUnregistered int
}
