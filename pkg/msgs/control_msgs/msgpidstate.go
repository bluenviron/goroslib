package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

type PidState struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	Header      std_msgs.Header //nolint:golint
	Timestep    time.Duration   //nolint:golint
	Error       float64         //nolint:golint
	ErrorDot    float64         //nolint:golint
	PError      float64         //nolint:golint
	IError      float64         //nolint:golint
	DError      float64         //nolint:golint
	PTerm       float64         //nolint:golint
	ITerm       float64         //nolint:golint
	DTerm       float64         //nolint:golint
	IMax        float64         //nolint:golint
	IMin        float64         //nolint:golint
	Output      float64         //nolint:golint
}
