//nolint:golint
package std_srvs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetBoolReq struct {
	Data bool
}

type SetBoolRes struct {
	Success bool
	Message string
}

type SetBool struct {
	msg.Package `ros:"std_srvs"`
	SetBoolReq
	SetBoolRes
}
