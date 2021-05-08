//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	Waypoint_FRAME_GLOBAL         uint8 = 0
	Waypoint_FRAME_LOCAL_NED      uint8 = 1
	Waypoint_FRAME_MISSION        uint8 = 2
	Waypoint_FRAME_GLOBAL_REL_ALT uint8 = 3
	Waypoint_FRAME_LOCAL_ENU      uint8 = 4
)

type Waypoint struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 FRAME_GLOBAL=0,uint8 FRAME_LOCAL_NED=1,uint8 FRAME_MISSION=2,uint8 FRAME_GLOBAL_REL_ALT=3,uint8 FRAME_LOCAL_ENU=4"`
	Frame           uint8
	Command         uint16
	IsCurrent       bool
	Autocontinue    bool
	Param1          float32
	Param2          float32
	Param3          float32
	Param4          float32
	XLat            float64
	YLong           float64
	ZAlt            float64
}
