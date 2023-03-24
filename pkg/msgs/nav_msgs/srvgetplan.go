//autogenerated:yes
//nolint:revive,lll
package nav_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs"
)

type GetPlanReq struct {
	msg.Package `ros:"nav_msgs"`
	Start       geometry_msgs.PoseStamped
	Goal        geometry_msgs.PoseStamped
	Tolerance   float32
}

type GetPlanRes struct {
	msg.Package `ros:"nav_msgs"`
	Plan        Path
}

type GetPlan struct {
	msg.Package `ros:"nav_msgs"`
	GetPlanReq
	GetPlanRes
}
