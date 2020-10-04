package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Pose2D struct {
	msgs.Package `ros:"geometry_msgs"`
	X            float64
	Y            float64
	Theta        float64
}
