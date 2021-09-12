//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LogRequestEndReq struct {
	msg.Package `ros:"mavros_msgs"`
}

type LogRequestEndRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
}

type LogRequestEnd struct {
	msg.Package `ros:"mavros_msgs"`
	LogRequestEndReq
	LogRequestEndRes
}
