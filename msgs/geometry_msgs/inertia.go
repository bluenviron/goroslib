package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Inertia struct {
	msgs.Package `ros:"geometry_msgs"`
	M            float64
	Com          Vector3
	Ixx          float64
	Ixy          float64
	Ixz          float64
	Iyy          float64
	Iyz          float64
	Izz          float64
}
