//nolint:golint
package diagnostic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	DiagnosticStatus_OK    int8 = 0
	DiagnosticStatus_WARN  int8 = 1
	DiagnosticStatus_ERROR int8 = 2
	DiagnosticStatus_STALE int8 = 3
)

type DiagnosticStatus struct {
	msg.Package     `ros:"diagnostic_msgs"`
	msg.Definitions `ros:"byte OK=0,byte WARN=1,byte ERROR=2,byte STALE=3"`
	Level           int8 `rostype:"byte"`
	Name            string
	Message         string
	HardwareId      string
	Values          []KeyValue
}
