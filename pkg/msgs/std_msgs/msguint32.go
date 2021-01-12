package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type UInt32 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        uint32 //nolint:golint
}
