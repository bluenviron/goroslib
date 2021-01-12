package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Pose struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Position    Point      //nolint:golint
	Orientation Quaternion //nolint:golint
}
