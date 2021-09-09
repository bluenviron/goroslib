//nolint:golint,lll
package std_srvs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TriggerReq struct{}

type TriggerRes struct {
	Success bool
	Message string
}

type Trigger struct {
	msg.Package `ros:"std_srvs"`
	TriggerReq
	TriggerRes
}
