package std_srvs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type EmptyReq struct { //nolint:golint
}

type EmptyRes struct { //nolint:golint
}

type Empty struct { //nolint:golint
	msg.Package `ros:"std_srvs"`
	EmptyReq
	EmptyRes
}
