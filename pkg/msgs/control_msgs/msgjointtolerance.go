package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type JointTolerance struct { //nolint:golint
	msg.Package  `ros:"control_msgs"`
	Name         string  //nolint:golint
	Position     float64 //nolint:golint
	Velocity     float64 //nolint:golint
	Acceleration float64 //nolint:golint
}
