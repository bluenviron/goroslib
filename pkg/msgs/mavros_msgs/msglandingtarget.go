//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	LandingTarget_GLOBAL                  uint8 = 0
	LandingTarget_LOCAL_NED               uint8 = 2
	LandingTarget_MISSION                 uint8 = 3
	LandingTarget_GLOBAL_RELATIVE_ALT     uint8 = 4
	LandingTarget_LOCAL_ENU               uint8 = 5
	LandingTarget_GLOBAL_INT              uint8 = 6
	LandingTarget_GLOBAL_RELATIVE_ALT_INT uint8 = 7
	LandingTarget_LOCAL_OFFSET_NED        uint8 = 8
	LandingTarget_BODY_NED                uint8 = 9
	LandingTarget_BODY_OFFSET_NED         uint8 = 10
	LandingTarget_GLOBAL_TERRAIN_ALT      uint8 = 11
	LandingTarget_GLOBAL_TERRAIN_ALT_INT  uint8 = 12
	LandingTarget_LIGHT_BEACON            uint8 = 0
	LandingTarget_RADIO_BEACON            uint8 = 1
	LandingTarget_VISION_FIDUCIAL         uint8 = 2
	LandingTarget_VISION_OTHER            uint8 = 3
)

type LandingTarget struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 GLOBAL=0,uint8 LOCAL_NED=2,uint8 MISSION=3,uint8 GLOBAL_RELATIVE_ALT=4,uint8 LOCAL_ENU=5,uint8 GLOBAL_INT=6,uint8 GLOBAL_RELATIVE_ALT_INT=7,uint8 LOCAL_OFFSET_NED=8,uint8 BODY_NED=9,uint8 BODY_OFFSET_NED=10,uint8 GLOBAL_TERRAIN_ALT=11,uint8 GLOBAL_TERRAIN_ALT_INT=12,uint8 LIGHT_BEACON=0,uint8 RADIO_BEACON=1,uint8 VISION_FIDUCIAL=2,uint8 VISION_OTHER=3"`
	Header          std_msgs.Header
	TargetNum       uint8
	Frame           uint8
	Angle           [2]float32
	Distance        float32
	Size            [2]float32
	Pose            geometry_msgs.Pose
	Type            uint8
}
