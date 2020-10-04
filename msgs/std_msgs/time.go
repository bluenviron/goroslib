package std_msgs

import (
	"github.com/aler9/goroslib/msg"
	"time"
)

type Time struct {
	msg.Package `ros:"std_msgs"`
	Data        time.Time
}
