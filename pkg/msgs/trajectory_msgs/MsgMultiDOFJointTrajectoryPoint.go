package trajectory_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"time"
)

type MultiDOFJointTrajectoryPoint struct { //nolint:golint
	msg.Package   `ros:"trajectory_msgs"`
	Transforms    []geometry_msgs.Transform //nolint:golint
	Velocities    []geometry_msgs.Twist     //nolint:golint
	Accelerations []geometry_msgs.Twist     //nolint:golint
	TimeFromStart time.Duration             //nolint:golint
}
