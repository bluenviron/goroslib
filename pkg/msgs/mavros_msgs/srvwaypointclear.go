//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type WaypointClearReq struct{}

type WaypointClearRes struct {
	Success bool
}

type WaypointClear struct {
	msg.Package `ros:"mavros_msgs"`
	WaypointClearReq
	WaypointClearRes
}
