package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Accel struct {
	msg.Package `ros:"geometry_msgs"`
	Linear      Vector3
	Angular     Vector3
}
