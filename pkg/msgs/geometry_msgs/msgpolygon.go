package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Polygon struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	Points      []Point32 //nolint:golint
}
