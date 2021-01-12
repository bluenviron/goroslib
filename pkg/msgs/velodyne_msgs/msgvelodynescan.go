package velodyne_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type VelodyneScan struct { //nolint:golint
	msg.Package `ros:"velodyne_msgs"`
	Header      std_msgs.Header  //nolint:golint
	Packets     []VelodynePacket //nolint:golint
}
