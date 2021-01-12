package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Wrench struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Force       Vector3 //nolint:golint
	Torque      Vector3 //nolint:golint
}
