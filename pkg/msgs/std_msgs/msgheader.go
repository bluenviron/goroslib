package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Header struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Seq         uint32    //nolint:golint
	Stamp       time.Time //nolint:golint
	FrameId     string    //nolint:golint
}
