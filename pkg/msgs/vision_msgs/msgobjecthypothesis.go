package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ObjectHypothesis struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Id          string  //nolint:golint
	Score       float64 //nolint:golint
}
