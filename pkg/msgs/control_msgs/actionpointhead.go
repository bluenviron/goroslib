package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"time"
)

type PointHeadActionGoal struct { //nolint:golint
	Target        geometry_msgs.PointStamped //nolint:golint
	PointingAxis  geometry_msgs.Vector3      //nolint:golint
	PointingFrame string                     //nolint:golint
	MinDuration   time.Duration              //nolint:golint
	MaxVelocity   float64                    //nolint:golint
}

type PointHeadActionResult struct { //nolint:golint
}

type PointHeadActionFeedback struct { //nolint:golint
	PointingAngleError float64 //nolint:golint
}

type PointHeadAction struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	PointHeadActionGoal
	PointHeadActionResult
	PointHeadActionFeedback
}
