//autogenerated:yes
//nolint:revive,lll
package mavros_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type CommandTriggerControlReq struct {
	msg.Package   `ros:"mavros_msgs"`
	TriggerEnable bool
	SequenceReset bool
	TriggerPause  bool
}

type CommandTriggerControlRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
	Result      uint8
}

type CommandTriggerControl struct {
	msg.Package `ros:"mavros_msgs"`
	CommandTriggerControlReq
	CommandTriggerControlRes
}
