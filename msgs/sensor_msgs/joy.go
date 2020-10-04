package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type Joy struct {
	msgs.Package `ros:"sensor_msgs"`
	Header       std_msgs.Header
	Axes         []float32
	Buttons      []int32
}
