package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type PoseWithCovariance struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Pose        Pose        //nolint:golint
	Covariance  [36]float64 //nolint:golint
}
