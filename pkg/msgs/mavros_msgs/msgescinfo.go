//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type ESCInfo struct {
	msg.Package    `ros:"mavros_msgs"`
	Header         std_msgs.Header
	Counter        uint16
	Count          uint8
	ConnectionType uint8
	Info           uint8
	EscInfo        []ESCInfoItem
}
