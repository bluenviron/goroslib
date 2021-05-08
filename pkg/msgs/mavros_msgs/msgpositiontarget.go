//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	PositionTarget_FRAME_LOCAL_NED        uint8  = 1
	PositionTarget_FRAME_LOCAL_OFFSET_NED uint8  = 7
	PositionTarget_FRAME_BODY_NED         uint8  = 8
	PositionTarget_FRAME_BODY_OFFSET_NED  uint8  = 9
	PositionTarget_IGNORE_PX              uint16 = 1
	PositionTarget_IGNORE_PY              uint16 = 2
	PositionTarget_IGNORE_PZ              uint16 = 4
	PositionTarget_IGNORE_VX              uint16 = 8
	PositionTarget_IGNORE_VY              uint16 = 16
	PositionTarget_IGNORE_VZ              uint16 = 32
	PositionTarget_IGNORE_AFX             uint16 = 64
	PositionTarget_IGNORE_AFY             uint16 = 128
	PositionTarget_IGNORE_AFZ             uint16 = 256
	PositionTarget_FORCE                  uint16 = 512
	PositionTarget_IGNORE_YAW             uint16 = 1024
	PositionTarget_IGNORE_YAW_RATE        uint16 = 2048
)

type PositionTarget struct {
	msg.Package         `ros:"mavros_msgs"`
	msg.Definitions     `ros:"uint8 FRAME_LOCAL_NED=1,uint8 FRAME_LOCAL_OFFSET_NED=7,uint8 FRAME_BODY_NED=8,uint8 FRAME_BODY_OFFSET_NED=9,uint16 IGNORE_PX=1,uint16 IGNORE_PY=2,uint16 IGNORE_PZ=4,uint16 IGNORE_VX=8,uint16 IGNORE_VY=16,uint16 IGNORE_VZ=32,uint16 IGNORE_AFX=64,uint16 IGNORE_AFY=128,uint16 IGNORE_AFZ=256,uint16 FORCE=512,uint16 IGNORE_YAW=1024,uint16 IGNORE_YAW_RATE=2048"`
	Header              std_msgs.Header
	CoordinateFrame     uint8
	TypeMask            uint16
	Position            geometry_msgs.Point
	Velocity            geometry_msgs.Vector3
	AccelerationOrForce geometry_msgs.Vector3
	Yaw                 float32
	YawRate             float32
}
