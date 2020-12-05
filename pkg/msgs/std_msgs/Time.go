//nolint:golint
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Time struct {
	msg.Package `ros:"std_msgs"`
	Data        time.Time
}
