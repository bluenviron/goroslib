package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/msgs/trajectory_msgs"
)

type JointTrajectoryControllerState struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	Header      std_msgs.Header                      //nolint:golint
	JointNames  []string                             //nolint:golint
	Desired     trajectory_msgs.JointTrajectoryPoint //nolint:golint
	Actual      trajectory_msgs.JointTrajectoryPoint //nolint:golint
	Error       trajectory_msgs.JointTrajectoryPoint //nolint:golint
}
