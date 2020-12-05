//nolint:golint
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MultiArrayDimension struct {
	msg.Package `ros:"std_msgs"`
	Label       string
	Size        uint32
	Stride      uint32
}
