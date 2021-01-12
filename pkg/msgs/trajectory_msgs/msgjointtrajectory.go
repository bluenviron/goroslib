package trajectory_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type JointTrajectory struct { //nolint:golint
	msg.Package `ros:"trajectory_msgs"`
	Header      std_msgs.Header        //nolint:golint
	JointNames  []string               //nolint:golint
	Points      []JointTrajectoryPoint //nolint:golint
}
