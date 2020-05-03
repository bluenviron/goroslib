package api_param

type RequestHasParam struct {
	CallerId string
	Name     string
}

type ResponseHasParam struct {
	Code    int
	NameOut string
	Res     bool
}

type RequestGetParam struct {
	CallerId string
	Name     string
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

type RequestSetParamBool struct {
	CallerId string
	Name     string
	Val      bool
}

type RequestSetParamInt struct {
	CallerId string
	Name     string
	Val      int
}

type RequestSetParamString struct {
	CallerId string
	Name     string
	Val      string
}

type ResponseSetParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}
