package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Int16 struct {
	msg.Package `ros:"std_msgs"`
	Data        int16
}
