package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type UInt64 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        uint64 //nolint:golint
}
