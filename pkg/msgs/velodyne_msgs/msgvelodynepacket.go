package velodyne_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type VelodynePacket struct { //nolint:golint
	msg.Package `ros:"velodyne_msgs"`
	Stamp       time.Time   //nolint:golint
	Data        [1206]uint8 //nolint:golint
}
