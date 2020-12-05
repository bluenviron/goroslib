//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	Range_ULTRASOUND uint8 = 0
	Range_INFRARED   uint8 = 1
)

type Range struct {
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"uint8 ULTRASOUND=0,uint8 INFRARED=1"`
	Header          std_msgs.Header
	RadiationType   uint8
	FieldOfView     float32
	MinRange        float32
	MaxRange        float32
	Range           float32
}
