package shape_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MeshTriangle struct { //nolint:golint
	msg.Package   `ros:"shape_msgs"`
	VertexIndices [3]uint32 //nolint:golint
}
