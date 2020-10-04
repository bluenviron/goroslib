package std_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type Float32 struct {
	msgs.Package `ros:"std_msgs"`
	Data         float32
}
