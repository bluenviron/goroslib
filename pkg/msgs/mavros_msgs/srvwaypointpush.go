//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type WaypointPushReq struct {
	StartIndex uint16
	Waypoints  []Waypoint
}

type WaypointPushRes struct {
	Success      bool
	WpTransfered uint32
}

type WaypointPush struct {
	msg.Package `ros:"mavros_msgs"`
	WaypointPushReq
	WaypointPushRes
}
