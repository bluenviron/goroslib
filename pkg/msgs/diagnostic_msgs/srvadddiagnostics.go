//nolint:golint
package diagnostic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AddDiagnosticsReq struct {
	LoadNamespace string
}

type AddDiagnosticsRes struct {
	Success bool
	Message string
}

type AddDiagnostics struct {
	msg.Package `ros:"diagnostic_msgs"`
	AddDiagnosticsReq
	AddDiagnosticsRes
}
