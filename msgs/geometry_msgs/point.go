package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Point struct {
	msgs.Package `ros:"geometry_msgs"`
	X            float64
	Y            float64
	Z            float64
}
