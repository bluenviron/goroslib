package diagnostic_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type DiagnosticArray struct { //nolint:golint
	msg.Package `ros:"diagnostic_msgs"`
	Header      std_msgs.Header    //nolint:golint
	Status      []DiagnosticStatus //nolint:golint
}
