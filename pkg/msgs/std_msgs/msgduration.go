//nolint:golint
package std_msgs

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
)

type Duration struct {
	msg.Package `ros:"std_msgs"`
	Data        time.Duration
}
