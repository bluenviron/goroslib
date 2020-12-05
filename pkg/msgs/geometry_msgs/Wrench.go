//nolint:golint
package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Wrench struct {
	msg.Package `ros:"geometry_msgs"`
	Force       Vector3
	Torque      Vector3
}
