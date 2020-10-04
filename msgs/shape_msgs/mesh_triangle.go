package shape_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type MeshTriangle struct {
	msg.Package   `ros:"shape_msgs"`
	VertexIndices [3]uint32
}
