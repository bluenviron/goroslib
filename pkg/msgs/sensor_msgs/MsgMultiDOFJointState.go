package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type MultiDOFJointState struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header           //nolint:golint
	JointNames  []string                  //nolint:golint
	Transforms  []geometry_msgs.Transform //nolint:golint
	Twist       []geometry_msgs.Twist     //nolint:golint
	Wrench      []geometry_msgs.Wrench    //nolint:golint
}
