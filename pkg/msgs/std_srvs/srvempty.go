//nolint:golint
package std_srvs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type EmptyReq struct{}

type EmptyRes struct{}

type Empty struct {
	msg.Package `ros:"std_srvs"`
	EmptyReq
	EmptyRes
}
