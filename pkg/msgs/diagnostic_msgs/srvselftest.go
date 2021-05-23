//nolint:golint
package diagnostic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SelfTestReq struct{}

type SelfTestRes struct {
	Id     string
	Passed int8 `rostype:"byte"`
	Status []DiagnosticStatus
}

type SelfTest struct {
	msg.Package `ros:"diagnostic_msgs"`
	SelfTestReq
	SelfTestRes
}
