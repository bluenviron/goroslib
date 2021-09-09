//nolint:golint,lll
package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Vector3Stamped struct {
	msg.Package `ros:"geometry_msgs"`
	Header      std_msgs.Header
	Vector      Vector3
}
