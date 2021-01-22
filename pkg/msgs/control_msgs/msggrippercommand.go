package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GripperCommand struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	Position    float64 //nolint:golint
	MaxEffort   float64 //nolint:golint
}
