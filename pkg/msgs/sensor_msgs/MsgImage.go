package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Image struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header //nolint:golint
	Height      uint32          //nolint:golint
	Width       uint32          //nolint:golint
	Encoding    string          //nolint:golint
	IsBigendian uint8           //nolint:golint
	Step        uint32          //nolint:golint
	Data        []uint8         //nolint:golint
}
