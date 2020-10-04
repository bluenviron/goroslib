package visualization_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type InteractiveMarkerFeedback struct {
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 KEEP_ALIVE=0,uint8 POSE_UPDATE=1,uint8 MENU_SELECT=2,uint8 BUTTON_CLICK=3,uint8 MOUSE_DOWN=4,uint8 MOUSE_UP=5"`
	Header          std_msgs.Header
	ClientId        string
	MarkerName      string
	ControlName     string
	EventType       uint8
	Pose            geometry_msgs.Pose
	MenuEntryId     uint32
	MousePoint      geometry_msgs.Point
	MousePointValid bool
}
