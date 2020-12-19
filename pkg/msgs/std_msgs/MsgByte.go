package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Byte struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        int8 `rostype:"byte"` //nolint:golint
}
