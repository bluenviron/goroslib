package std_srvs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TriggerReq struct { //nolint:golint
	msg.Package `ros:"std_srvs"`
}

type TriggerRes struct { //nolint:golint
	msg.Package `ros:"std_srvs"`
	Success     bool   //nolint:golint
	Message     string //nolint:golint
}
