package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

type SingleJointPositionActionGoal struct { //nolint:golint
	Position    float64       //nolint:golint
	MinDuration time.Duration //nolint:golint
	MaxVelocity float64       //nolint:golint
}

type SingleJointPositionActionResult struct { //nolint:golint
}

type SingleJointPositionActionFeedback struct { //nolint:golint
	Header   std_msgs.Header //nolint:golint
	Position float64         //nolint:golint
	Velocity float64         //nolint:golint
	Error    float64         //nolint:golint
}

type SingleJointPositionAction struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	SingleJointPositionActionGoal
	SingleJointPositionActionResult
	SingleJointPositionActionFeedback
}
