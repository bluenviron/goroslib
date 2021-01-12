package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Twist struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Linear      Vector3 //nolint:golint
	Angular     Vector3 //nolint:golint
}
