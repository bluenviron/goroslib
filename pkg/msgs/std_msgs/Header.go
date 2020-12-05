//nolint:golint
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Header struct {
	msg.Package `ros:"std_msgs"`
	Seq         uint32
	Stamp       time.Time
	FrameId     string
}
