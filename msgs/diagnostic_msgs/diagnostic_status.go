package diagnostic_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type DiagnosticStatus struct {
	msg.Package     `ros:"diagnostic_msgs"`
	msg.Definitions `ros:"byte OK=0,byte WARN=1,byte ERROR=2,byte STALE=3"`
	Level           int8 `ros:"byte"`
	Name            string
	Message         string
	HardwareId      string
	Values          []KeyValue
}
