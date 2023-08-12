//autogenerated:yes
//nolint:revive,lll
package nav_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type GetMapActionGoal struct {
	msg.Package `ros:"nav_msgs"`
}

type GetMapActionResult struct {
	msg.Package `ros:"nav_msgs"`
	Map         OccupancyGrid
}

type GetMapActionFeedback struct {
	msg.Package `ros:"nav_msgs"`
}

type GetMapAction struct {
	msg.Package `ros:"nav_msgs"`
	GetMapActionGoal
	GetMapActionResult
	GetMapActionFeedback
}