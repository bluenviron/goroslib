package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type UInt8 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        uint8 //nolint:golint
}
