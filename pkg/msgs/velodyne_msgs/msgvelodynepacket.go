//nolint:golint
package velodyne_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type VelodynePacket struct {
	msg.Package `ros:"velodyne_msgs"`
	Stamp       time.Time
	Data        [1206]uint8
}
