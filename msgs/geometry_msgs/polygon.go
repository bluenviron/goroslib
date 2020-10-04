package geometry_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type Polygon struct {
	msg.Package `ros:"geometry_msgs"`
	Points      []Point32
}
