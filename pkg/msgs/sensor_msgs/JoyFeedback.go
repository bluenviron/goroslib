package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	JoyFeedback_TYPE_LED    uint8 = 0
	JoyFeedback_TYPE_RUMBLE uint8 = 1
	JoyFeedback_TYPE_BUZZER uint8 = 2
)

type JoyFeedback struct {
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"uint8 TYPE_LED=0,uint8 TYPE_RUMBLE=1,uint8 TYPE_BUZZER=2"`
	Type            uint8
	Id              uint8
	Intensity       float32
}
