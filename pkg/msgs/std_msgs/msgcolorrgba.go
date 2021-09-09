//nolint:golint,lll
package std_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ColorRGBA struct {
	msg.Package `ros:"std_msgs"`
	R           float32
	G           float32
	B           float32
	A           float32
}
