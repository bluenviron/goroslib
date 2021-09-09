//nolint:golint,lll
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Odometry struct {
	msg.Package  `ros:"nav_msgs"`
	Header       std_msgs.Header
	ChildFrameId string
	Pose         geometry_msgs.PoseWithCovariance
	Twist        geometry_msgs.TwistWithCovariance
}
