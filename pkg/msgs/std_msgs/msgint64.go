package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Int64 struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        int64 //nolint:golint
}
