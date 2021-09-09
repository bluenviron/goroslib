//nolint:golint,lll
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GripperCommandActionGoal struct {
	Command GripperCommand
}

type GripperCommandActionResult struct {
	Position    float64
	Effort      float64
	Stalled     bool
	ReachedGoal bool
}

type GripperCommandActionFeedback struct {
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
