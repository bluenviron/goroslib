package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type GetPlanReq struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Start       geometry_msgs.PoseStamped //nolint:golint
	Goal        geometry_msgs.PoseStamped //nolint:golint
	Tolerance   float32                   //nolint:golint
}

type GetPlanRes struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Plan        Path //nolint:golint
}
