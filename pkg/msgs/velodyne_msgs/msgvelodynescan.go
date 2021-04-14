//nolint:golint
package velodyne_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type VelodyneScan struct {
	msg.Package `ros:"velodyne_msgs"`
	Header      std_msgs.Header
	Packets     []VelodynePacket
}
