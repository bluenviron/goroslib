package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"time"
)

type TimeReference struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header //nolint:golint
	TimeRef     time.Time       //nolint:golint
	Source      string          //nolint:golint
}
