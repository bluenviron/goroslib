//nolint:golint
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapReq struct{}

type GetMapRes struct {
	Map OccupancyGrid
}

type GetMap struct {
	msg.Package `ros:"nav_msgs"`
	GetMapReq
	GetMapRes
}
