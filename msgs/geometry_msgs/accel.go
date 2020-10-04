package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Accel struct {
	msgs.Package `ros:"geometry_msgs"`
	Linear       Vector3
	Angular      Vector3
}
