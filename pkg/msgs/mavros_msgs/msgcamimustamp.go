//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type CamIMUStamp struct {
	msg.Package `ros:"mavros_msgs"`
	FrameStamp  time.Time
	FrameSeqId  int32
}
