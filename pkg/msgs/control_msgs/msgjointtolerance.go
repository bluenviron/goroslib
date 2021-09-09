//nolint:golint,lll
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type JointTolerance struct {
	msg.Package  `ros:"control_msgs"`
	Name         string
	Position     float64
	Velocity     float64
	Acceleration float64
}
