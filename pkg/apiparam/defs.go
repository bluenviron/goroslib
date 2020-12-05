// Package apiparam implements the Parameter API.
//
// https://wiki.ros.org/ROS/Parameter%20Server%20API
package apiparam

type RequestDeleteParam struct {
	CallerID string
	Key      string
}

type ResponseDeleteParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}

type RequestGetParamNames struct {
	CallerID string
}

type ResponseGetParamNames struct {
	Code          int
	StatusMessage string
	List          []string
}

type RequestGetParam struct {
	CallerID string
	Key      string
}

type ResponseGetParamBool struct {
	Code          int
	StatusMessage string
	Res           bool
}

type ResponseGetParamInt struct {
	Code          int
	StatusMessage string
	Res           int
}

type ResponseGetParamString struct {
	Code          int
	StatusMessage string
	Res           string
}

type RequestHasParam struct {
	CallerID string
	Key      string
}

type ResponseHasParam struct {
	Code   int
	KeyOut string
	Res    bool
}

type RequestSearchParam struct {
	CallerID string
	Key      string
}

type ResponseSearchParam struct {
	Code          int
	StatusMessage string
	FoundKey      string
}

type RequestSetParamBool struct {
	CallerID string
	Key      string
	Val      bool
}

type RequestSetParamInt struct {
	CallerID string
	Key      string
	Val      int
}

type RequestSetParamString struct {
	CallerID string
	Key      string
	Val      string
}

type ResponseSetParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}
