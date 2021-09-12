//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ParamPushReq struct {
	msg.Package `ros:"mavros_msgs"`
}

type ParamPushRes struct {
	msg.Package     `ros:"mavros_msgs"`
	Success         bool
	ParamTransfered uint32
}

type ParamPush struct {
	msg.Package `ros:"mavros_msgs"`
	ParamPushReq
	ParamPushRes
}
