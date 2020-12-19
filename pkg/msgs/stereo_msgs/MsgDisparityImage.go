package stereo_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type DisparityImage struct { //nolint:golint
	msg.Package  `ros:"stereo_msgs"`
	Header       std_msgs.Header              //nolint:golint
	Image        sensor_msgs.Image            //nolint:golint
	F            float32                      //nolint:golint
	T            float32                      `rosname:"T"` //nolint:golint
	ValidWindow  sensor_msgs.RegionOfInterest //nolint:golint
	MinDisparity float32                      //nolint:golint
	MaxDisparity float32                      //nolint:golint
	DeltaD       float32                      //nolint:golint
}
