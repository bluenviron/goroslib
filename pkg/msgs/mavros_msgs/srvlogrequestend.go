//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LogRequestEndReq struct{}

type LogRequestEndRes struct {
	Success bool
}

type LogRequestEnd struct {
	msg.Package `ros:"mavros_msgs"`
	LogRequestEndReq
	LogRequestEndRes
}
