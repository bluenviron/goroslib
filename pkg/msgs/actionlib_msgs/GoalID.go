package actionlib_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type GoalID struct {
	msg.Package `ros:"actionlib_msgs"`
	Stamp       time.Time
	Id          string
}
