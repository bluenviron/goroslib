package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Int64MultiArray struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Layout      MultiArrayLayout //nolint:golint
	Data        []int64          //nolint:golint
}
