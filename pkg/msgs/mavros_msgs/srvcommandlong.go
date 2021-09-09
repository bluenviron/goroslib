//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandLongReq struct {
	Broadcast    bool
	Command      uint16
	Confirmation uint8
	Param1       float32
	Param2       float32
	Param3       float32
	Param4       float32
	Param5       float32
	Param6       float32
	Param7       float32
}

type CommandLongRes struct {
	Success bool
	Result  uint8
}

type CommandLong struct {
	msg.Package `ros:"mavros_msgs"`
	CommandLongReq
	CommandLongRes
}
