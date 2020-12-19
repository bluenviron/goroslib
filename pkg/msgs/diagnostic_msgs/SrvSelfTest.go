//nolint:golint
package diagnostic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SelfTestReq struct {
	msg.Package `ros:"diagnostic_msgs"`
}

type SelfTestRes struct {
	msg.Package `ros:"diagnostic_msgs"`
	Id          string
	Passed      int8 `rostype:"byte"`
	Status      []DiagnosticStatus
}
