//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	SetMavFrameReq_FRAME_GLOBAL                  uint8 = 0
	SetMavFrameReq_FRAME_LOCAL_NED               uint8 = 1
	SetMavFrameReq_FRAME_MISSION                 uint8 = 2
	SetMavFrameReq_FRAME_GLOBAL_RELATIVE_ALT     uint8 = 3
	SetMavFrameReq_FRAME_LOCAL_ENU               uint8 = 4
	SetMavFrameReq_FRAME_GLOBAL_INT              uint8 = 5
	SetMavFrameReq_FRAME_GLOBAL_RELATIVE_ALT_INT uint8 = 6
	SetMavFrameReq_FRAME_LOCAL_OFFSET_NED        uint8 = 7
	SetMavFrameReq_FRAME_BODY_NED                uint8 = 8
	SetMavFrameReq_FRAME_BODY_OFFSET_NED         uint8 = 9
	SetMavFrameReq_FRAME_GLOBAL_TERRAIN_ALT      uint8 = 10
	SetMavFrameReq_FRAME_GLOBAL_TERRAIN_ALT_INT  uint8 = 11
	SetMavFrameReq_FRAME_BODY_FRD                uint8 = 12
	SetMavFrameReq_FRAME_RESERVED_13             uint8 = 13
	SetMavFrameReq_FRAME_RESERVED_14             uint8 = 14
	SetMavFrameReq_FRAME_RESERVED_15             uint8 = 15
	SetMavFrameReq_FRAME_RESERVED_16             uint8 = 16
	SetMavFrameReq_FRAME_RESERVED_17             uint8 = 17
	SetMavFrameReq_FRAME_RESERVED_18             uint8 = 18
	SetMavFrameReq_FRAME_RESERVED_19             uint8 = 19
	SetMavFrameReq_FRAME_LOCAL_FRD               uint8 = 20
	SetMavFrameReq_FRAME_LOCAL_FLU               uint8 = 21
)

type SetMavFrameReq struct {
	msg.Definitions `ros:"uint8 FRAME_GLOBAL=0,uint8 FRAME_LOCAL_NED=1,uint8 FRAME_MISSION=2,uint8 FRAME_GLOBAL_RELATIVE_ALT=3,uint8 FRAME_LOCAL_ENU=4,uint8 FRAME_GLOBAL_INT=5,uint8 FRAME_GLOBAL_RELATIVE_ALT_INT=6,uint8 FRAME_LOCAL_OFFSET_NED=7,uint8 FRAME_BODY_NED=8,uint8 FRAME_BODY_OFFSET_NED=9,uint8 FRAME_GLOBAL_TERRAIN_ALT=10,uint8 FRAME_GLOBAL_TERRAIN_ALT_INT=11,uint8 FRAME_BODY_FRD=12,uint8 FRAME_RESERVED_13=13,uint8 FRAME_RESERVED_14=14,uint8 FRAME_RESERVED_15=15,uint8 FRAME_RESERVED_16=16,uint8 FRAME_RESERVED_17=17,uint8 FRAME_RESERVED_18=18,uint8 FRAME_RESERVED_19=19,uint8 FRAME_LOCAL_FRD=20,uint8 FRAME_LOCAL_FLU=21"`
	MavFrame        uint8
}

type SetMavFrameRes struct {
	Success bool
}

type SetMavFrame struct {
	msg.Package `ros:"mavros_msgs"`
	SetMavFrameReq
	SetMavFrameRes
}
