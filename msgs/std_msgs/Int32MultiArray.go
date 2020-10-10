package std_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type Int32MultiArray struct {
	msg.Package `ros:"std_msgs"`
	Layout      MultiArrayLayout
	Data        []int32
}
