package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type RegionOfInterest struct {
	msg.Package `ros:"sensor_msgs"`
	XOffset     uint32
	YOffset     uint32
	Height      uint32
	Width       uint32
	DoRectify   bool
}
