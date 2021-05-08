//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandBoolReq struct {
	Value bool
}

type CommandBoolRes struct {
	Success bool
	Result  uint8
}

type CommandBool struct {
	msg.Package `ros:"mavros_msgs"`
	CommandBoolReq
	CommandBoolRes
}
