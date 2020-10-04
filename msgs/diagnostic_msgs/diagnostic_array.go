package diagnostic_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type DiagnosticArray struct {
	msg.Package `ros:"diagnostic_msgs"`
	Header      std_msgs.Header
	Status      []DiagnosticStatus
}
