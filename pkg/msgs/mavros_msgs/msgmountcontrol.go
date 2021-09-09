//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	MountControl_MAV_MOUNT_MODE_RETRACT           uint8 = 0
	MountControl_MAV_MOUNT_MODE_NEUTRAL           uint8 = 1
	MountControl_MAV_MOUNT_MODE_MAVLINK_TARGETING uint8 = 2
	MountControl_MAV_MOUNT_MODE_RC_TARGETING      uint8 = 3
	MountControl_MAV_MOUNT_MODE_GPS_POINT         uint8 = 4
)

type MountControl struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 MAV_MOUNT_MODE_RETRACT=0,uint8 MAV_MOUNT_MODE_NEUTRAL=1,uint8 MAV_MOUNT_MODE_MAVLINK_TARGETING=2,uint8 MAV_MOUNT_MODE_RC_TARGETING=3,uint8 MAV_MOUNT_MODE_GPS_POINT=4"`
	Header          std_msgs.Header
	Mode            uint8
	Pitch           float32
	Roll            float32
	Yaw             float32
	Altitude        float32
	Latitude        float32
	Longitude       float32
}
