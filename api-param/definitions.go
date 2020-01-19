package api_param

type hasParamReq struct {
	CallerId string
	Name     string
}

type HasParamRes struct {
	Code    int
	NameOut string
	Res     bool
}

type getParamReq struct {
	CallerId string
	Name     string
}

type getParamBoolRes struct {
	Code          int
	StatusMessage string
	Res           bool
}

type getParamIntRes struct {
	Code          int
	StatusMessage string
	Res           int
}

type getParamStringRes struct {
	Code          int
	StatusMessage string
	Res           string
}

type setParamBoolReq struct {
	CallerId string
	Name     string
	Val      bool
}

type setParamIntReq struct {
	CallerId string
	Name     string
	Val      int
}

type setParamStringReq struct {
	CallerId string
	Name     string
	Val      string
}

type setParamRes struct {
	Code          int
	StatusMessage string
	Ignore        int
}
