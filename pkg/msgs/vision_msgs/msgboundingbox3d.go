package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type BoundingBox3D struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Center      geometry_msgs.Pose    //nolint:golint
	Size        geometry_msgs.Vector3 //nolint:golint
}
