package geometry_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type PolygonStamped struct {
	msg.Package `ros:"geometry_msgs"`
	Header      std_msgs.Header
	Polygon     Polygon
}
