package visualization_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type MenuEntry struct {
	msg.Package     `ros:"visualization_msgs"`
	msg.Definitions `ros:"uint8 FEEDBACK=0,uint8 ROSRUN=1,uint8 ROSLAUNCH=2"`
	Id              uint32
	ParentId        uint32
	Title           string
	Command         string
	CommandType     uint8
}
