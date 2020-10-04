package geometry_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type Point32 struct {
	msg.Package `ros:"geometry_msgs"`
	X           float32
	Y           float32
	Z           float32
}
