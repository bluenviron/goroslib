//nolint:golint,lll
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/trajectory_msgs"
)

type JointTrajectoryActionGoal struct {
	msg.Package `ros:"control_msgs"`
	Trajectory  trajectory_msgs.JointTrajectory
}

type JointTrajectoryActionResult struct {
	msg.Package `ros:"control_msgs"`
}

type JointTrajectoryActionFeedback struct {
	msg.Package `ros:"control_msgs"`
}

type JointTrajectoryAction struct {
	msg.Package `ros:"control_msgs"`
	JointTrajectoryActionGoal
	JointTrajectoryActionResult
	JointTrajectoryActionFeedback
}
