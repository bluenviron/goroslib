//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MessageIntervalReq struct {
	msg.Package `ros:"mavros_msgs"`
	MessageId   uint32
	MessageRate float32
}

type MessageIntervalRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
}

type MessageInterval struct {
	msg.Package `ros:"mavros_msgs"`
	MessageIntervalReq
	MessageIntervalRes
}
