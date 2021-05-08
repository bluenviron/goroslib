//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	OverrideRCIn_CHAN_RELEASE  uint16 = 0
	OverrideRCIn_CHAN_NOCHANGE uint16 = 65535
)

type OverrideRCIn struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint16 CHAN_RELEASE=0,uint16 CHAN_NOCHANGE=65535"`
	Channels        [8]uint16
}
