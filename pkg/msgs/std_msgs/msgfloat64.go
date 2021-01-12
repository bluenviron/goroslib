package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Float64 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        float64 //nolint:golint
}
