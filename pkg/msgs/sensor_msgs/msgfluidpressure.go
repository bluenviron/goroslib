package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type FluidPressure struct { //nolint:golint
	msg.Package   `ros:"sensor_msgs"`
	Header        std_msgs.Header //nolint:golint
	FluidPressure float64         //nolint:golint
	Variance      float64         //nolint:golint
}
