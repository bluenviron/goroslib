package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Int32 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        int32 //nolint:golint
}
