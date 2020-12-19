//nolint:golint
package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Pose struct {
	msg.Package `ros:"geometry_msgs"`
	Position    Point
	Orientation Quaternion
}
