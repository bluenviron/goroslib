//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Altitude struct {
	msg.Package     `ros:"mavros_msgs"`
	Header          std_msgs.Header
	Monotonic       float32
	Amsl            float32
	Local           float32
	Relative        float32
	Terrain         float32
	BottomClearance float32
}
