package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type JoyFeedbackArray struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Array       []JoyFeedback //nolint:golint
}
