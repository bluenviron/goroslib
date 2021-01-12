package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Detection3D struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Header      std_msgs.Header            //nolint:golint
	Results     []ObjectHypothesisWithPose //nolint:golint
	Bbox        BoundingBox3D              //nolint:golint
	SourceCloud sensor_msgs.PointCloud2    //nolint:golint
	IsTracking  bool                       //nolint:golint
	TrackingId  string                     //nolint:golint
}
