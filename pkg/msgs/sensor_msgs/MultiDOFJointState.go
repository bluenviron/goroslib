//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type MultiDOFJointState struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	JointNames  []string
	Transforms  []geometry_msgs.Transform
	Twist       []geometry_msgs.Twist
	Wrench      []geometry_msgs.Wrench
}
