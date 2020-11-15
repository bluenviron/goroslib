package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Joy struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	Axes        []float32
	Buttons     []int32
}
