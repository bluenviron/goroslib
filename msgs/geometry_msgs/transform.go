package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Transform struct {
	msgs.Package `ros:"geometry_msgs"`
	Translation  Vector3
	Rotation     Quaternion
}
