//nolint:golint
package shape_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type Mesh struct {
	msg.Package `ros:"shape_msgs"`
	Triangles   []MeshTriangle
	Vertices    []geometry_msgs.Point
}
