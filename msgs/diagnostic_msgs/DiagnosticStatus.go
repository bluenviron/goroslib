package diagnostic_msgs

import (
	"github.com/aler9/goroslib/msg"
)

const (
	DiagnosticStatus_OK    int8 = 0
	DiagnosticStatus_WARN  int8 = 1
	DiagnosticStatus_ERROR int8 = 2
	DiagnosticStatus_STALE int8 = 3
)

type DiagnosticStatus struct {
	msg.Package     `ros:"diagnostic_msgs"`
	msg.Definitions `ros:"int8 OK=0,int8 WARN=1,int8 ERROR=2,int8 STALE=3"`
	Level           int8 `ros:"byte"`
	Name            string
	Message         string
	HardwareId      string
	Values          []KeyValue
}
