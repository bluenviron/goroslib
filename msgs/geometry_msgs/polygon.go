package geometry_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Polygon struct {
	msgs.Package `ros:"geometry_msgs"`
	Points       []Point32
}
