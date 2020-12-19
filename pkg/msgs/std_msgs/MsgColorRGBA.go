package std_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ColorRGBA struct { //nolint:golint
	msg.Package `ros:"std_msgs"`
	R           float32 //nolint:golint
	G           float32 //nolint:golint
	B           float32 //nolint:golint
	A           float32 //nolint:golint
}
