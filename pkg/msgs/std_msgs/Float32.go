package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Float32 struct {
	msg.Package `ros:"std_msgs"`
	Data        float32
}
