package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type RegionOfInterest struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	XOffset     uint32 //nolint:golint
	YOffset     uint32 //nolint:golint
	Height      uint32 //nolint:golint
	Width       uint32 //nolint:golint
	DoRectify   bool   //nolint:golint
}
