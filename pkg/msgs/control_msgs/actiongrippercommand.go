//nolint:golint,lll
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GripperCommandActionGoal struct {
	msg.Package `ros:"control_msgs"`
	Command     GripperCommand
}

type GripperCommandActionResult struct {
	msg.Package `ros:"control_msgs"`
	Position    float64
	Effort      float64
	Stalled     bool
	ReachedGoal bool
}

type GripperCommandActionFeedback struct {
	msg.Package `ros:"control_msgs"`
	Position    float64
	Effort      float64
	Stalled     bool
	ReachedGoal bool
}

type GripperCommandAction struct {
	msg.Package `ros:"control_msgs"`
	GripperCommandActionGoal
	GripperCommandActionResult
	GripperCommandActionFeedback
}
