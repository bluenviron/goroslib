package actionlib_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type GoalStatus struct {
	msg.Package     `ros:"actionlib_msgs"`
	msg.Definitions `ros:"uint8 PENDING=0,uint8 ACTIVE=1,uint8 PREEMPTED=2,uint8 SUCCEEDED=3,uint8 ABORTED=4,uint8 REJECTED=5,uint8 PREEMPTING=6,uint8 RECALLING=7,uint8 RECALLED=8,uint8 LOST=9"`
	GoalId          GoalID
	Status          uint8
	Text            string
}
