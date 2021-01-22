package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/trajectory_msgs"
)

type JointTrajectoryActionGoal struct { //nolint:golint
	Trajectory trajectory_msgs.JointTrajectory //nolint:golint
}

type JointTrajectoryActionResult struct { //nolint:golint
}

type JointTrajectoryActionFeedback struct { //nolint:golint
}

type JointTrajectoryAction struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	JointTrajectoryActionGoal
	JointTrajectoryActionResult
	JointTrajectoryActionFeedback
}
