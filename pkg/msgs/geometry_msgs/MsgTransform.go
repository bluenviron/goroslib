package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Transform struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Translation Vector3    //nolint:golint
	Rotation    Quaternion //nolint:golint
}
