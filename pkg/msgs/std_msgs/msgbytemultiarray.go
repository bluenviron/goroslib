//nolint:golint,lll
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ByteMultiArray struct {
	msg.Package `ros:"std_msgs"`
	Layout      MultiArrayLayout
	Data        []int8 `rostype:"byte"`
}
