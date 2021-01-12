package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ChannelFloat32 struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Name        string    //nolint:golint
	Values      []float32 //nolint:golint
}
