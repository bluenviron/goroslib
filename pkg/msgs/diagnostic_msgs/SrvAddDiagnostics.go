package diagnostic_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AddDiagnosticsReq struct { //nolint:golint
	msg.Package   `ros:"diagnostic_msgs"`
	LoadNamespace string //nolint:golint
}

type AddDiagnosticsRes struct { //nolint:golint
	msg.Package `ros:"diagnostic_msgs"`
	Success     bool   //nolint:golint
	Message     string //nolint:golint
}
