package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"time"
)

type MapMetaData struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	MapLoadTime time.Time          //nolint:golint
	Resolution  float32            //nolint:golint
	Width       uint32             //nolint:golint
	Height      uint32             //nolint:golint
	Origin      geometry_msgs.Pose //nolint:golint
}
