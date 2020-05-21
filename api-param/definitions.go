package api_param

type RequestDeleteParam struct {
	CallerId string
	Key      string
}

type ResponseDeleteParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}

type RequestGetParamNames struct {
	CallerId string
}

type ResponseGetParamNames struct {
	Code          int
	StatusMessage string
	List          []string
}

type RequestGetParam struct {
	CallerId string
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
	CallerId string
	Key      string
}

type ResponseHasParam struct {
	Code   int
	KeyOut string
	Res    bool
}

type RequestSearchParam struct {
	CallerId string
	Key      string
}

type ResponseSearchParam struct {
	Code          int
	StatusMessage string
	FoundKey      string
}

type RequestSetParamBool struct {
	CallerId string
	Key      string
	Val      bool
}

type RequestSetParamInt struct {
	CallerId string
	Key      string
	Val      int
}

type RequestSetParamString struct {
	CallerId string
	Key      string
	Val      string
}

type ResponseSetParam struct {
	Code          int
	StatusMessage string
	Ignore        int
}
