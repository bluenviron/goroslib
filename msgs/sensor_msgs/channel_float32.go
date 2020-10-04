package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type ChannelFloat32 struct {
	msgs.Package `ros:"sensor_msgs"`
	Name         string
	Values       []float32
}
