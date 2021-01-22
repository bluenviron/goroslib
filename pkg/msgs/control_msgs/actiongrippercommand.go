package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GripperCommandActionGoal struct { //nolint:golint
	Command GripperCommand //nolint:golint
}

type GripperCommandActionResult struct { //nolint:golint
	Position    float64 //nolint:golint
	Effort      float64 //nolint:golint
	Stalled     bool    //nolint:golint
	ReachedGoal bool    //nolint:golint
}

type GripperCommandActionFeedback struct { //nolint:golint
	Position    float64 //nolint:golint
	Effort      float64 //nolint:golint
	Stalled     bool    //nolint:golint
	ReachedGoal bool    //nolint:golint
}

type GripperCommandAction struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	GripperCommandActionGoal
	GripperCommandActionResult
	GripperCommandActionFeedback
}
