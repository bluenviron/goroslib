package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type RegionOfInterest struct {
	msgs.Package `ros:"sensor_msgs"`
	XOffset      uint32
	YOffset      uint32
	Height       uint32
	Width        uint32
	DoRectify    bool
}
