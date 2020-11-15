package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Duration struct {
	msg.Package `ros:"std_msgs"`
	Data        time.Duration
}
