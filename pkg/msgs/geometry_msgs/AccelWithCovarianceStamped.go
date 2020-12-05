//nolint:golint
package geometry_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type AccelWithCovarianceStamped struct {
	msg.Package `ros:"geometry_msgs"`
	Header      std_msgs.Header
	Accel       AccelWithCovariance
}
