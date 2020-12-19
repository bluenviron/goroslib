package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type PointCloud struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header         //nolint:golint
	Points      []geometry_msgs.Point32 //nolint:golint
	Channels    []ChannelFloat32        //nolint:golint
}
