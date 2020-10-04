package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type PoseWithCovariance struct {
	msgs.Package `ros:"geometry_msgs"`
	Pose         Pose
	Covariance   [36]float64
}
