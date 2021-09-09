//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandTriggerControlReq struct {
	TriggerEnable bool
	SequenceReset bool
	TriggerPause  bool
}

type CommandTriggerControlRes struct {
	Success bool
	Result  uint8
}

type CommandTriggerControl struct {
	msg.Package `ros:"mavros_msgs"`
	CommandTriggerControlReq
	CommandTriggerControlRes
}
