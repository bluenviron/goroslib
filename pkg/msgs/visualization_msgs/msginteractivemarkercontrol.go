package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
)

const (
	InteractiveMarkerControl_INHERIT        uint8 = 0 //nolint:golint
	InteractiveMarkerControl_FIXED          uint8 = 1 //nolint:golint
	InteractiveMarkerControl_VIEW_FACING    uint8 = 2 //nolint:golint
	InteractiveMarkerControl_NONE           uint8 = 0 //nolint:golint
	InteractiveMarkerControl_MENU           uint8 = 1 //nolint:golint
	InteractiveMarkerControl_BUTTON         uint8 = 2 //nolint:golint
	InteractiveMarkerControl_MOVE_AXIS      uint8 = 3 //nolint:golint
	InteractiveMarkerControl_MOVE_PLANE     uint8 = 4 //nolint:golint
	InteractiveMarkerControl_ROTATE_AXIS    uint8 = 5 //nolint:golint
	InteractiveMarkerControl_MOVE_ROTATE    uint8 = 6 //nolint:golint
	InteractiveMarkerControl_MOVE_3D        uint8 = 7 //nolint:golint
	InteractiveMarkerControl_ROTATE_3D      uint8 = 8 //nolint:golint
	InteractiveMarkerControl_MOVE_ROTATE_3D uint8 = 9 //nolint:golint
)

type InteractiveMarkerControl struct { //nolint:golint
	msg.Package                  `ros:"visualization_msgs"`
	msg.Definitions              `ros:"uint8 INHERIT=0,uint8 FIXED=1,uint8 VIEW_FACING=2,uint8 NONE=0,uint8 MENU=1,uint8 BUTTON=2,uint8 MOVE_AXIS=3,uint8 MOVE_PLANE=4,uint8 ROTATE_AXIS=5,uint8 MOVE_ROTATE=6,uint8 MOVE_3D=7,uint8 ROTATE_3D=8,uint8 MOVE_ROTATE_3D=9"`
	Name                         string                   //nolint:golint
	Orientation                  geometry_msgs.Quaternion //nolint:golint
	OrientationMode              uint8                    //nolint:golint
	InteractionMode              uint8                    //nolint:golint
	AlwaysVisible                bool                     //nolint:golint
	Markers                      []Marker                 //nolint:golint
	IndependentMarkerOrientation bool                     //nolint:golint
	Description                  string                   //nolint:golint
}
