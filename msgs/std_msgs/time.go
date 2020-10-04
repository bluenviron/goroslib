package std_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"time"
)

type Time struct {
	msgs.Package `ros:"std_msgs"`
	Data         time.Time
}
