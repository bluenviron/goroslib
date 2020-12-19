package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type SetMapReq struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Map         OccupancyGrid                           //nolint:golint
	InitialPose geometry_msgs.PoseWithCovarianceStamped //nolint:golint
}

type SetMapRes struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Success     bool //nolint:golint
}
