package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	MenuEntry_FEEDBACK  uint8 = 0 //nolint:golint
	MenuEntry_ROSRUN    uint8 = 1 //nolint:golint
	MenuEntry_ROSLAUNCH uint8 = 2 //nolint:golint
)

type MenuEntry struct { //nolint:golint
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 FEEDBACK=0,uint8 ROSRUN=1,uint8 ROSLAUNCH=2"`
	Id              uint32 //nolint:golint
	ParentId        uint32 //nolint:golint
	Title           string //nolint:golint
	Command         string //nolint:golint
	CommandType     uint8  //nolint:golint
}
