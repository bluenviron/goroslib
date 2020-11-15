package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TwistWithCovariance struct {
	msg.Package `ros:"geometry_msgs"`
	Twist       Twist
	Covariance  [36]float64
}
