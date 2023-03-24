//autogenerated:yes
//nolint:revive,lll
package sensor_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
)

type PointCloud struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	Points      []geometry_msgs.Point32
	Channels    []ChannelFloat32
}
