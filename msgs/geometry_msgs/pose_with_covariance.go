package geometry_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type PoseWithCovariance struct {
	msg.Package `ros:"geometry_msgs"`
	Pose        Pose
	Covariance  [36]float64
}
