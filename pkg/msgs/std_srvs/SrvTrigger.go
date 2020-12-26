package std_srvs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type TriggerReq struct { //nolint:golint
}

type TriggerRes struct { //nolint:golint
	Success bool   //nolint:golint
	Message string //nolint:golint
}

type Trigger struct { //nolint:golint
	msg.Package `ros:"std_srvs"`
	TriggerReq
	TriggerRes
}
