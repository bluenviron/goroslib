package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	JoyFeedback_TYPE_LED    uint8 = 0 //nolint:golint
	JoyFeedback_TYPE_RUMBLE uint8 = 1 //nolint:golint
	JoyFeedback_TYPE_BUZZER uint8 = 2 //nolint:golint
)

type JoyFeedback struct { //nolint:golint
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"uint8 TYPE_LED=0,uint8 TYPE_RUMBLE=1,uint8 TYPE_BUZZER=2"`
	Type            uint8   //nolint:golint
	Id              uint8   //nolint:golint
	Intensity       float32 //nolint:golint
}
