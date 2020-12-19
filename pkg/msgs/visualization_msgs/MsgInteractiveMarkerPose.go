package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type InteractiveMarkerPose struct { //nolint:golint
	msg.Package `ros:"visualization_msgs"`
	Header      std_msgs.Header    //nolint:golint
	Pose        geometry_msgs.Pose //nolint:golint
	Name        string             //nolint:golint
}
