package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type String struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        string //nolint:golint
}
