package nav_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type OccupancyGrid struct {
	msg.Package `ros:"nav_msgs"`
	Header      std_msgs.Header
	Info        MapMetaData
	Data        []int8
}
