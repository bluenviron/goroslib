package nav_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type OccupancyGrid struct {
	msgs.Package `ros:"nav_msgs"`
	Header       std_msgs.Header
	Info         MapMetaData
	Data         []int8
}
