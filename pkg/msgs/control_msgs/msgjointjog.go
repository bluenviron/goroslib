//nolint:golint
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type JointJog struct {
	msg.Package   `ros:"control_msgs"`
	Header        std_msgs.Header
	JointNames    []string
	Displacements []float64
	Velocities    []float64
	Duration      float64
}
