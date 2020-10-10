package geometry_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type Transform struct {
	msg.Package `ros:"geometry_msgs"`
	Translation Vector3
	Rotation    Quaternion
}
