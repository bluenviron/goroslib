package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Duration struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        time.Duration //nolint:golint
}
