package diagnostic_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SelfTestReq struct { //nolint:golint
}

type SelfTestRes struct { //nolint:golint
	Id     string             //nolint:golint
	Passed int8               `rostype:"byte"` //nolint:golint
	Status []DiagnosticStatus //nolint:golint
}

type SelfTest struct { //nolint:golint
	msg.Package `ros:"diagnostic_msgs"`
	SelfTestReq
	SelfTestRes
}
