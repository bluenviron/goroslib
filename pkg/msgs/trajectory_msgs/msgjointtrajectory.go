//nolint:golint,lll
package trajectory_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type JointTrajectory struct {
	msg.Package `ros:"trajectory_msgs"`
	Header      std_msgs.Header
	JointNames  []string
	Points      []JointTrajectoryPoint
}
