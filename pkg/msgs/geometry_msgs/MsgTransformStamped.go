package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type TransformStamped struct { //nolint:golint
	msg.Package  `ros:"geometry_msgs"`
	Header       std_msgs.Header //nolint:golint
	ChildFrameId string          //nolint:golint
	Transform    Transform       //nolint:golint
}
