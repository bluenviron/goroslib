package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type JointState struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header //nolint:golint
	Name        []string        //nolint:golint
	Position    []float64       //nolint:golint
	Velocity    []float64       //nolint:golint
	Effort      []float64       //nolint:golint
}
