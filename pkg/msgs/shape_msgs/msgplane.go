package shape_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Plane struct { //nolint:golint
	msg.Package `ros:"shape_msgs"`
	Coef        [4]float64 //nolint:golint
}
