//nolint:golint
package velodyne_msgs

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
)

type VelodynePacket struct {
	msg.Package `ros:"velodyne_msgs"`
	Stamp       time.Time
	Data        [1206]uint8
}
