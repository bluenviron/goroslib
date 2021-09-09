//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandIntReq struct {
	Broadcast    bool
	Frame        uint8
	Command      uint16
	Current      uint8
	Autocontinue uint8
	Param1       float32
	Param2       float32
	Param3       float32
	Param4       float32
	X            int32
	Y            int32
	Z            float32
}

type CommandIntRes struct {
	Success bool
}

type CommandInt struct {
	msg.Package `ros:"mavros_msgs"`
	CommandIntReq
	CommandIntRes
}
