package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Detection2D struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Header      std_msgs.Header            //nolint:golint
	Results     []ObjectHypothesisWithPose //nolint:golint
	Bbox        BoundingBox2D              //nolint:golint
	SourceImg   sensor_msgs.Image          //nolint:golint
	IsTracking  bool                       //nolint:golint
	TrackingId  string                     //nolint:golint
}
