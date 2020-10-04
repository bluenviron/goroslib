package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type Image struct {
	msgs.Package `ros:"sensor_msgs"`
	Header       std_msgs.Header
	Height       uint32
	Width        uint32
	Encoding     string
	IsBigendian  uint8
	Step         uint32
	Data         []uint8
}
