//nolint:golint
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Byte struct {
	msg.Package `ros:"std_msgs"`
	Data        int8 `rostype:"byte"`
}
