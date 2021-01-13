package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type GetMapActionGoal struct { //nolint:golint
}

type GetMapActionResult struct { //nolint:golint
	Map OccupancyGrid //nolint:golint
}

type GetMapActionFeedback struct { //nolint:golint
}

type GetMapAction struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	GetMapActionGoal
	GetMapActionResult
	GetMapActionFeedback
}
