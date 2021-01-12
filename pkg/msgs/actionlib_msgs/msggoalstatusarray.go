package actionlib_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type GoalStatusArray struct { //nolint:golint
	msg.Package `ros:"actionlib_msgs"`
	Header      std_msgs.Header //nolint:golint
	StatusList  []GoalStatus    //nolint:golint
}
