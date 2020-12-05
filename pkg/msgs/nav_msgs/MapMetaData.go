//nolint:golint
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"time"
)

type MapMetaData struct {
	msg.Package `ros:"nav_msgs"`
	MapLoadTime time.Time
	Resolution  float32
	Width       uint32
	Height      uint32
	Origin      geometry_msgs.Pose
}
