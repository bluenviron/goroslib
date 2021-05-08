//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	Mavlink_FRAMING_OK            uint8 = 1
	Mavlink_FRAMING_BAD_CRC       uint8 = 2
	Mavlink_FRAMING_BAD_SIGNATURE uint8 = 3
	Mavlink_MAVLINK_V10           uint8 = 254
	Mavlink_MAVLINK_V20           uint8 = 253
)

type Mavlink struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 FRAMING_OK=1,uint8 FRAMING_BAD_CRC=2,uint8 FRAMING_BAD_SIGNATURE=3,uint8 MAVLINK_V10=254,uint8 MAVLINK_V20=253"`
	Header          std_msgs.Header
	FramingStatus   uint8
	Magic           uint8
	Len             uint8
	IncompatFlags   uint8
	CompatFlags     uint8
	Seq             uint8
	Sysid           uint8
	Compid          uint8
	Msgid           uint32
	Checksum        uint16
	Payload64       []uint64
	Signature       []uint8
}
