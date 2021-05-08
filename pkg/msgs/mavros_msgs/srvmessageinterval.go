//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MessageIntervalReq struct {
	MessageId   uint32
	MessageRate float32
}

type MessageIntervalRes struct {
	Success bool
}

type MessageInterval struct {
	msg.Package `ros:"mavros_msgs"`
	MessageIntervalReq
	MessageIntervalRes
}
