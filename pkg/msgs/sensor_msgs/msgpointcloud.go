//nolint:golint,lll
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type PointCloud struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	Points      []geometry_msgs.Point32
	Channels    []ChannelFloat32
}
