//nolint:golint,lll
package diagnostic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AddDiagnosticsReq struct {
	msg.Package   `ros:"diagnostic_msgs"`
	LoadNamespace string
}

type AddDiagnosticsRes struct {
	msg.Package `ros:"diagnostic_msgs"`
	Success     bool
	Message     string
}

type AddDiagnostics struct {
	msg.Package `ros:"diagnostic_msgs"`
	AddDiagnosticsReq
	AddDiagnosticsRes
}
