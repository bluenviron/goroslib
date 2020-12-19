package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type AccelWithCovariance struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Accel       Accel       //nolint:golint
	Covariance  [36]float64 //nolint:golint
}
