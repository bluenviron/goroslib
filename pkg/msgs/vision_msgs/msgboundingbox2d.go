package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type BoundingBox2D struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Center      geometry_msgs.Pose2D //nolint:golint
	SizeX       float64              //nolint:golint
	SizeY       float64              //nolint:golint
}
