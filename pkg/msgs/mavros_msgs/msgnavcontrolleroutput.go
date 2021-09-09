//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type NavControllerOutput struct {
	msg.Package   `ros:"mavros_msgs"`
	Header        std_msgs.Header
	NavRoll       float32
	NavPitch      float32
	NavBearing    int16
	TargetBearing int16
	WpDist        uint16
	AltError      float32
	AspdError     float32
	XtrackError   float32
}
