package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Char struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Data        uint8 `rostype:"char"` //nolint:golint
}
