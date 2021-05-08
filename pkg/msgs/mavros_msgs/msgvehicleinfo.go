//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	VehicleInfo_HAVE_INFO_HEARTBEAT         uint8 = 1
	VehicleInfo_HAVE_INFO_AUTOPILOT_VERSION uint8 = 2
)

type VehicleInfo struct {
	msg.Package         `ros:"mavros_msgs"`
	msg.Definitions     `ros:"uint8 HAVE_INFO_HEARTBEAT=1,uint8 HAVE_INFO_AUTOPILOT_VERSION=2"`
	Header              std_msgs.Header
	AvailableInfo       uint8
	Sysid               uint8
	Compid              uint8
	Autopilot           uint8
	Type                uint8
	SystemStatus        uint8
	BaseMode            uint8
	CustomMode          uint32
	Mode                string
	ModeId              uint32
	Capabilities        uint64
	FlightSwVersion     uint32
	MiddlewareSwVersion uint32
	OsSwVersion         uint32
	BoardVersion        uint32
	FlightCustomVersion string
	VendorId            uint16
	ProductId           uint16
	Uid                 uint64
}
