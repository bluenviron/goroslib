//nolint:golint
package std_srvs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetBoolReq struct {
	msg.Package `ros:"std_srvs"`
	Data        bool
}

type SetBoolRes struct {
	msg.Package `ros:"std_srvs"`
	Success     bool
	Message     string
}
