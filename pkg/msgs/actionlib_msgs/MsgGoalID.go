package actionlib_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type GoalID struct { //nolint:golint
	msg.Package `ros:"actionlib_msgs"`
	Stamp       time.Time //nolint:golint
	Id          string    //nolint:golint
}
