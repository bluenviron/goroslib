package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type ObjectHypothesisWithPose struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Id          string                           //nolint:golint
	Score       float64                          //nolint:golint
	Pose        geometry_msgs.PoseWithCovariance //nolint:golint
}
