package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MultiArrayDimension struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	Label       string //nolint:golint
	Size        uint32 //nolint:golint
	Stride      uint32 //nolint:golint
}
