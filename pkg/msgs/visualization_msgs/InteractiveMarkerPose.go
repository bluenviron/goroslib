//nolint:golint
package visualization_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type InteractiveMarkerPose struct {
	msg.Package `ros:"visualization_msgs"`
	Header      std_msgs.Header
	Pose        geometry_msgs.Pose
	Name        string
}
