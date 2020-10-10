package std_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type Char struct {
	msg.Package `ros:"std_msgs"`
	Data        uint8 `ros:"char"`
}
