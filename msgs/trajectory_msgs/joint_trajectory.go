package trajectory_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type JointTrajectory struct {
	msgs.Package `ros:"trajectory_msgs"`
	Header       std_msgs.Header
	JointNames   []string
	Points       []JointTrajectoryPoint
}
