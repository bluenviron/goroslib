package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type CompressedImage struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header //nolint:golint
	Format      string          //nolint:golint
	Data        []uint8         //nolint:golint
}
