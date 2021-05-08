//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ParamPullReq struct {
	ForcePull bool
}

type ParamPullRes struct {
	Success       bool
	ParamReceived uint32
}

type ParamPull struct {
	msg.Package `ros:"mavros_msgs"`
	ParamPullReq
	ParamPullRes
}
