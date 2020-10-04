package std_msgs

import (
	"github.com/aler9/goroslib/msgs"
)

type MultiArrayLayout struct {
	msgs.Package `ros:"std_msgs"`
	Dim          []MultiArrayDimension
	DataOffset   uint32
}
