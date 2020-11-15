package shape_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MeshTriangle struct {
	msg.Package   `ros:"shape_msgs"`
	VertexIndices [3]uint32
}
