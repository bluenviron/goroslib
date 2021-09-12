//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ParamSetReq struct {
	msg.Package `ros:"mavros_msgs"`
	ParamId     string
	Value       ParamValue
}

type ParamSetRes struct {
	msg.Package `ros:"mavros_msgs"`
	Success     bool
	Value       ParamValue
}

type ParamSet struct {
	msg.Package `ros:"mavros_msgs"`
	ParamSetReq
	ParamSetRes
}
