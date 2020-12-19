package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type InteractiveMarker struct { //nolint:golint
	msg.Package `ros:"visualization_msgs"`
	Header      std_msgs.Header            //nolint:golint
	Pose        geometry_msgs.Pose         //nolint:golint
	Name        string                     //nolint:golint
	Description string                     //nolint:golint
	Scale       float32                    //nolint:golint
	MenuEntries []MenuEntry                //nolint:golint
	Controls    []InteractiveMarkerControl //nolint:golint
}
