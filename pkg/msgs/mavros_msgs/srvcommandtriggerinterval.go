//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandTriggerIntervalReq struct {
	CycleTime       float32
	IntegrationTime float32
}

type CommandTriggerIntervalRes struct {
	Success bool
	Result  uint8
}

type CommandTriggerInterval struct {
	msg.Package `ros:"mavros_msgs"`
	CommandTriggerIntervalReq
	CommandTriggerIntervalRes
}
