//nolint:golint
package stereo_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/sensor_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type DisparityImage struct {
	msg.Package  `ros:"stereo_msgs"`
	Header       std_msgs.Header
	Image        sensor_msgs.Image
	F            float32
	T            float32 `rosname:"T"`
	ValidWindow  sensor_msgs.RegionOfInterest
	MinDisparity float32
	MaxDisparity float32
	DeltaD       float32
}
