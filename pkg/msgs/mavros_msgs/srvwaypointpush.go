//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type WaypointPushReq struct {
	msg.Package `ros:"mavros_msgs"`
	StartIndex  uint16
	Waypoints   []Waypoint
}

type WaypointPushRes struct {
	msg.Package  `ros:"mavros_msgs"`
	Success      bool
	WpTransfered uint32
}

type WaypointPush struct {
	msg.Package `ros:"mavros_msgs"`
	WaypointPushReq
	WaypointPushRes
}
