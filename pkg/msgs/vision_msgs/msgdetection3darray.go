package vision_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Detection3DArray struct { //nolint:golint
	msg.Package `ros:"vision_msgs"`
	Header      std_msgs.Header //nolint:golint
	Detections  []Detection3D   //nolint:golint
}
