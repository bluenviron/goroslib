package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Bool struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        bool //nolint:golint
}
