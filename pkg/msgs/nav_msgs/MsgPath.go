package nav_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Path struct { //nolint:golint
	msg.Package `ros:"nav_msgs"`
	Header      std_msgs.Header             //nolint:golint
	Poses       []geometry_msgs.PoseStamped //nolint:golint
}
