//nolint:golint
package control_msgs

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type PointHeadActionGoal struct {
	Target        geometry_msgs.PointStamped
	PointingAxis  geometry_msgs.Vector3
	PointingFrame string
	MinDuration   time.Duration
	MaxVelocity   float64
}

type PointHeadActionResult struct{}

type PointHeadActionFeedback struct {
	PointingAngleError float64
}

type PointHeadAction struct {
	msg.Package `ros:"control_msgs"`
	PointHeadActionGoal
	PointHeadActionResult
	PointHeadActionFeedback
}
