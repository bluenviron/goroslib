package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapReq struct { //nolint:golint
}

type GetMapRes struct { //nolint:golint
	Map OccupancyGrid //nolint:golint
}

type GetMap struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	GetMapReq
	GetMapRes
}
