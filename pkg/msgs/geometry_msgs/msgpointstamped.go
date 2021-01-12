package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type PointStamped struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Header      std_msgs.Header //nolint:golint
	Point       Point           //nolint:golint
}
