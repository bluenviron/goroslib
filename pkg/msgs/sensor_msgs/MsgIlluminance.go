package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Illuminance struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header //nolint:golint
	Illuminance float64         //nolint:golint
	Variance    float64         //nolint:golint
}
