package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type PoseWithCovarianceStamped struct {
	msgs.Package `ros:"geometry_msgs"`
	Header       std_msgs.Header
	Pose         PoseWithCovariance
}
