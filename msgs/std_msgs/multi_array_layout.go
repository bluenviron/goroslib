package std_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type MultiArrayLayout struct {
	msg.Package `ros:"std_msgs"`
	Dim         []MultiArrayDimension
	DataOffset  uint32
}
