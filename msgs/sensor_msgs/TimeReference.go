package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
	"time"
)

type TimeReference struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	TimeRef     time.Time
	Source      string
}
