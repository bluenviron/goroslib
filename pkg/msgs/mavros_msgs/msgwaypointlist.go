//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type WaypointList struct {
	msg.Package `ros:"mavros_msgs"`
	CurrentSeq  uint16
	Waypoints   []Waypoint
}
