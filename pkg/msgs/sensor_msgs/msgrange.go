package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	Range_ULTRASOUND uint8 = 0 //nolint:golint
	Range_INFRARED   uint8 = 1 //nolint:golint
)

type Range struct { //nolint:golint
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"uint8 ULTRASOUND=0,uint8 INFRARED=1"`
	Header          std_msgs.Header //nolint:golint
	RadiationType   uint8           //nolint:golint
	FieldOfView     float32         //nolint:golint
	MinRange        float32         //nolint:golint
	MaxRange        float32         //nolint:golint
	Range           float32         //nolint:golint
}
