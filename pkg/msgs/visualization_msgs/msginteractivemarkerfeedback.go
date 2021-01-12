package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	InteractiveMarkerFeedback_KEEP_ALIVE   uint8 = 0 //nolint:golint
	InteractiveMarkerFeedback_POSE_UPDATE  uint8 = 1 //nolint:golint
	InteractiveMarkerFeedback_MENU_SELECT  uint8 = 2 //nolint:golint
	InteractiveMarkerFeedback_BUTTON_CLICK uint8 = 3 //nolint:golint
	InteractiveMarkerFeedback_MOUSE_DOWN   uint8 = 4 //nolint:golint
	InteractiveMarkerFeedback_MOUSE_UP     uint8 = 5 //nolint:golint
)

type InteractiveMarkerFeedback struct { //nolint:golint
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 KEEP_ALIVE=0,uint8 POSE_UPDATE=1,uint8 MENU_SELECT=2,uint8 BUTTON_CLICK=3,uint8 MOUSE_DOWN=4,uint8 MOUSE_UP=5"`
	Header          std_msgs.Header     //nolint:golint
	ClientId        string              //nolint:golint
	MarkerName      string              //nolint:golint
	ControlName     string              //nolint:golint
	EventType       uint8               //nolint:golint
	Pose            geometry_msgs.Pose  //nolint:golint
	MenuEntryId     uint32              //nolint:golint
	MousePoint      geometry_msgs.Point //nolint:golint
	MousePointValid bool                //nolint:golint
}
