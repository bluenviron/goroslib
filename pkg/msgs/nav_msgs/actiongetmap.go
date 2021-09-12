//nolint:golint,lll
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
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
