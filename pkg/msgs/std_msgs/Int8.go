//nolint:golint
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Int8 struct {
	msg.Package `ros:"std_msgs"`
	Data        int8
}
