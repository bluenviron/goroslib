package diagnostic_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	DiagnosticStatus_OK    int8 = 0 //nolint:golint
	DiagnosticStatus_WARN  int8 = 1 //nolint:golint
	DiagnosticStatus_ERROR int8 = 2 //nolint:golint
	DiagnosticStatus_STALE int8 = 3 //nolint:golint
)

type DiagnosticStatus struct { //nolint:golint
	msg.Package     `ros:"diagnostic_msgs"`
	msg.Definitions `ros:"byte OK=0,byte WARN=1,byte ERROR=2,byte STALE=3"`
	Level           int8       `rostype:"byte"` //nolint:golint
	Name            string     //nolint:golint
	Message         string     //nolint:golint
	HardwareId      string     //nolint:golint
	Values          []KeyValue //nolint:golint
}
