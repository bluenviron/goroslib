//nolint:golint
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapReq struct {
	msg.Package `ros:"nav_msgs"`
}

type GetMapRes struct {
	msg.Package `ros:"nav_msgs"`
	Map         OccupancyGrid
}
