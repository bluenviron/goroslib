//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandAckReq struct {
	Command      uint16
	Result       uint8
	Progress     uint8
	ResultParam2 uint32
}

type CommandAckRes struct {
	Success bool
	Result  uint8
}

type CommandAck struct {
	msg.Package `ros:"mavros_msgs"`
	CommandAckReq
	CommandAckRes
}
