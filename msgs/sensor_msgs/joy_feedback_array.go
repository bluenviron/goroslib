package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type JoyFeedbackArray struct {
	msgs.Package `ros:"sensor_msgs"`
	Array        []JoyFeedback
}
