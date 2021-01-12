package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type BoundingBox3DArray struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Header      std_msgs.Header //nolint:golint
	Boxes       []BoundingBox3D //nolint:golint
}
