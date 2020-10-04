package nav_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type Path struct {
	msg.Package `ros:"nav_msgs"`
	Header      std_msgs.Header
	Poses       []geometry_msgs.PoseStamped
}
