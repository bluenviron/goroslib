package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type GetPlanReq struct { //nolint:golint
	Start     geometry_msgs.PoseStamped //nolint:golint
	Goal      geometry_msgs.PoseStamped //nolint:golint
	Tolerance float32                   //nolint:golint
}

type GetPlanRes struct { //nolint:golint
	Plan Path //nolint:golint
}

type GetPlan struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	GetPlanReq
	GetPlanRes
}
