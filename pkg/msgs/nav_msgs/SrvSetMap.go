package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type SetMapReq struct { //nolint:golint
	Map         OccupancyGrid                           //nolint:golint
	InitialPose geometry_msgs.PoseWithCovarianceStamped //nolint:golint
}

type SetMapRes struct { //nolint:golint
	Success bool //nolint:golint
}

type SetMap struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	SetMapReq
	SetMapRes
}
