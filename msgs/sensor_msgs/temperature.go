package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type Temperature struct {
	msgs.Package `ros:"sensor_msgs"`
	Header       std_msgs.Header
	Temperature  float64
	Variance     float64
}
