//nolint:golint
package nav_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapActionGoal struct {
}

type GetMapActionResult struct {
	Map OccupancyGrid
}

type GetMapActionFeedback struct {
}

type GetMapAction struct {
	msg.Package `ros:"nav_msgs"`
	GetMapActionGoal
	GetMapActionResult
	GetMapActionFeedback
}
