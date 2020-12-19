package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapReq struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
}

type GetMapRes struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Map         OccupancyGrid //nolint:golint
}
