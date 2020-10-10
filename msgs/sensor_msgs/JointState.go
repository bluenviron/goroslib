package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type JointState struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	Name        []string
	Position    []float64
	Velocity    []float64
	Effort      []float64
}
