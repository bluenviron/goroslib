//nolint:golint
package actionlib_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type GoalStatusArray struct {
	msg.Package `ros:"actionlib_msgs"`
	Header      std_msgs.Header
	StatusList  []GoalStatus
}
