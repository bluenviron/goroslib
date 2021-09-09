//nolint:golint,lll
package vision_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type BoundingBox3D struct {
	msg.Package `ros:"vision_msgs"`
	Center      geometry_msgs.Pose
	Size        geometry_msgs.Vector3
}
