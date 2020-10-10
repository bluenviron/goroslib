package geometry_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type Accel struct {
	msg.Package `ros:"geometry_msgs"`
	Linear      Vector3
	Angular     Vector3
}
