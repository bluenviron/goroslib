package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type JoyFeedbackArray struct {
	msg.Package `ros:"sensor_msgs"`
	Array       []JoyFeedback
}
