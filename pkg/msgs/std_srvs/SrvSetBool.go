package std_srvs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetBoolReq struct { //nolint:golint
	msg.Package `ros:"std_srvs"`
	Data        bool //nolint:golint
}

type SetBoolRes struct { //nolint:golint
	msg.Package `ros:"std_srvs"`
	Success     bool   //nolint:golint
	Message     string //nolint:golint
}
