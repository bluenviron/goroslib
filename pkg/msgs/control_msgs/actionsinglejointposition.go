//nolint:golint,lll
package control_msgs

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type SingleJointPositionActionGoal struct {
	msg.Package `ros:"control_msgs"`
	Position    float64
	MinDuration time.Duration
	MaxVelocity float64
}

type SingleJointPositionActionResult struct {
	msg.Package `ros:"control_msgs"`
}

type SingleJointPositionActionFeedback struct {
	msg.Package `ros:"control_msgs"`
	Header      std_msgs.Header
	Position    float64
	Velocity    float64
	Error       float64
}

type SingleJointPositionAction struct {
	msg.Package `ros:"control_msgs"`
	SingleJointPositionActionGoal
	SingleJointPositionActionResult
	SingleJointPositionActionFeedback
}
