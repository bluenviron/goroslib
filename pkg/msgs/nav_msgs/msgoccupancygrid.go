package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type OccupancyGrid struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Header      std_msgs.Header //nolint:golint
	Info        MapMetaData     //nolint:golint
	Data        []int8          //nolint:golint
}
