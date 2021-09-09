//nolint:golint,lll
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GripperCommand struct {
	msg.Package `ros:"control_msgs"`
	Position    float64
	MaxEffort   float64
}
