//autogenerated:yes
//nolint:revive,lll
package mavros_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/std_msgs"
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
