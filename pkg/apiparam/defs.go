// Package apiparam implements the Parameter API.
//
// https://wiki.ros.org/ROS/Parameter%20Server%20API
package apiparam

// RequestDeleteParam is a deleteParam request.
type RequestDeleteParam struct {
	CallerID string
	Key      string
}

// ResponseDeleteParam is the response to a deleteParam request.
type ResponseDeleteParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}

// RequestGetParamNames is a getParamNames request.
type RequestGetParamNames struct {
	CallerID string
}

// ResponseGetParamNames is the response to a getParamNames request.
type ResponseGetParamNames struct {
	Code          int
	StatusMessage string
	List          []string
}

// RequestGetParam is a getParam request.
type RequestGetParam struct {
	CallerID string
	Key      string
}

// ResponseGetParamBool is the response to a getParam request.
type ResponseGetParamBool struct {
	Code          int
	StatusMessage string
	Res           bool
}

// ResponseGetParamInt is the response to a getParam request.
type ResponseGetParamInt struct {
	Code          int
	StatusMessage string
	Res           int
}

// ResponseGetParamString is the response to a getParam request.
type ResponseGetParamString struct {
	Code          int
	StatusMessage string
	Res           string
}

// RequestHasParam is a hasParam request.
type RequestHasParam struct {
	CallerID string
	Key      string
}

// ResponseHasParam is the response to a hasParam request.
type ResponseHasParam struct {
	Code   int
	KeyOut string
	Res    bool
}

// RequestSearchParam is a searchParam request.
type RequestSearchParam struct {
	CallerID string
	Key      string
}

// ResponseSearchParam is the response to a searchParam request.
type ResponseSearchParam struct {
	Code          int
	StatusMessage string
	FoundKey      string
}

// RequestParamSetBool is a setParam request.
type RequestParamSetBool struct {
	CallerID string
	Key      string
	Val      bool
}

// RequestParamSetInt is a setParam request.
type RequestParamSetInt struct {
	CallerID string
	Key      string
	Val      int
}

// RequestParamSetString is a setParam request.
type RequestParamSetString struct {
	CallerID string
	Key      string
	Val      string
}

// ResponseSetParam is the response to a setParam request.
type ResponseSetParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}
