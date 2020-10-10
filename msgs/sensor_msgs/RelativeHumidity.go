package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type RelativeHumidity struct {
	msg.Package      `ros:"sensor_msgs"`
	Header           std_msgs.Header
	RelativeHumidity float64
	Variance         float64
}
