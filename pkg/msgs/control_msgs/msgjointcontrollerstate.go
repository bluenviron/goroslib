package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type JointControllerState struct { //nolint:golint
	msg.Package     `ros:"control_msgs"`
	Header          std_msgs.Header //nolint:golint
	SetPoint        float64         //nolint:golint
	ProcessValue    float64         //nolint:golint
	ProcessValueDot float64         //nolint:golint
	Error           float64         //nolint:golint
	TimeStep        float64         //nolint:golint
	Command         float64         //nolint:golint
	P               float64         //nolint:golint
	I               float64         //nolint:golint
	D               float64         //nolint:golint
	IClamp          float64         //nolint:golint
	Antiwindup      bool            //nolint:golint
}
