package std_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type String struct {
	msg.Package `ros:"std_msgs"`
	Data        string
}
