package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Classification2D struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Header      std_msgs.Header    //nolint:golint
	Results     []ObjectHypothesis //nolint:golint
	SourceImg   sensor_msgs.Image  //nolint:golint
}
