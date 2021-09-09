//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LogRequestListReq struct {
	Start uint16
	End   uint16
}

type LogRequestListRes struct {
	Success bool
}

type LogRequestList struct {
	msg.Package `ros:"mavros_msgs"`
	LogRequestListReq
	LogRequestListRes
}
