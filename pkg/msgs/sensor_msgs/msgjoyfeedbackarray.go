//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type JoyFeedbackArray struct {
	msg.Package `ros:"sensor_msgs"`
	Array       []JoyFeedback
}
