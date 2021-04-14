//nolint:golint
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/trajectory_msgs"
)

type JointTrajectoryActionGoal struct {
	Trajectory trajectory_msgs.JointTrajectory
}

type JointTrajectoryActionResult struct {
}

type JointTrajectoryActionFeedback struct {
}

type JointTrajectoryAction struct {
	msg.Package `ros:"control_msgs"`
	JointTrajectoryActionGoal
	JointTrajectoryActionResult
	JointTrajectoryActionFeedback
}
