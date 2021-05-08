//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ParamPushReq struct {
}

type ParamPushRes struct {
	Success         bool
	ParamTransfered uint32
}

type ParamPush struct {
	msg.Package `ros:"mavros_msgs"`
	ParamPushReq
	ParamPushRes
}
