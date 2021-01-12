package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Point32 struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	X           float32 //nolint:golint
	Y           float32 //nolint:golint
	Z           float32 //nolint:golint
}
