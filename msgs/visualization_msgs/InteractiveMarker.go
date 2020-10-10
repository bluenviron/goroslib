package visualization_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type InteractiveMarker struct {
	msg.Package `ros:"visualization_msgs"`
	Header      std_msgs.Header
	Pose        geometry_msgs.Pose
	Name        string
	Description string
	Scale       float32
	MenuEntries []MenuEntry
	Controls    []InteractiveMarkerControl
}
