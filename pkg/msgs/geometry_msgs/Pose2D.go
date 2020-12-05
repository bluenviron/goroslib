//nolint:golint
package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Pose2D struct {
	msg.Package `ros:"geometry_msgs"`
	X           float64
	Y           float64
	Theta       float64
}
