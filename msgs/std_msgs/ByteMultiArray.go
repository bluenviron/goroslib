package std_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type ByteMultiArray struct {
	msg.Package `ros:"std_msgs"`
	Layout      MultiArrayLayout
	Data        []int8 `rostype:"byte"`
}
