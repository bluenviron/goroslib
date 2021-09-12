//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type WaypointPullReq struct {
	msg.Package `ros:"mavros_msgs"`
}

type WaypointPullRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
	WpReceived  uint32
}

type WaypointPull struct {
	msg.Package `ros:"mavros_msgs"`
	WaypointPullReq
	WaypointPullRes
}
