package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type JointJog struct { //nolint:golint
	msg.Package   `ros:"control_msgs"`
	Header        std_msgs.Header //nolint:golint
	JointNames    []string        //nolint:golint
	Displacements []float64       //nolint:golint
	Velocities    []float64       //nolint:golint
	Duration      float64         //nolint:golint
}
