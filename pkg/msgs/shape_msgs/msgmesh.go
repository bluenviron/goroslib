package shape_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type Mesh struct { //nolint:golint
	msg.Package `ros:"shape_msgs"`
	Triangles   []MeshTriangle        //nolint:golint
	Vertices    []geometry_msgs.Point //nolint:golint
}
