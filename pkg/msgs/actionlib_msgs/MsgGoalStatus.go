package actionlib_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	GoalStatus_PENDING    uint8 = 0 //nolint:golint
	GoalStatus_ACTIVE     uint8 = 1 //nolint:golint
	GoalStatus_PREEMPTED  uint8 = 2 //nolint:golint
	GoalStatus_SUCCEEDED  uint8 = 3 //nolint:golint
	GoalStatus_ABORTED    uint8 = 4 //nolint:golint
	GoalStatus_REJECTED   uint8 = 5 //nolint:golint
	GoalStatus_PREEMPTING uint8 = 6 //nolint:golint
	GoalStatus_RECALLING  uint8 = 7 //nolint:golint
	GoalStatus_RECALLED   uint8 = 8 //nolint:golint
	GoalStatus_LOST       uint8 = 9 //nolint:golint
)

type GoalStatus struct { //nolint:golint
	msg.Package     `ros:"actionlib_msgs"`
	msg.Definitions `ros:"uint8 PENDING=0,uint8 ACTIVE=1,uint8 PREEMPTED=2,uint8 SUCCEEDED=3,uint8 ABORTED=4,uint8 REJECTED=5,uint8 PREEMPTING=6,uint8 RECALLING=7,uint8 RECALLED=8,uint8 LOST=9"`
	GoalId          GoalID //nolint:golint
	Status          uint8  //nolint:golint
	Text            string //nolint:golint
}
