//nolint:golint
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

type GeoPose struct {
	msg.Package `ros:"geographic_msgs"`
	Position    GeoPoint
	Orientation geometry_msgs.Quaternion
}
