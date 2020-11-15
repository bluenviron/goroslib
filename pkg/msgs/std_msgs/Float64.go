package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Float64 struct {
	msg.Package `ros:"std_msgs"`
	Data        float64
}
