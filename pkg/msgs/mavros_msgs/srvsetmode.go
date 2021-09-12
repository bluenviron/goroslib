//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	SetModeReq_MAV_MODE_PREFLIGHT          uint8 = 0
	SetModeReq_MAV_MODE_STABILIZE_DISARMED uint8 = 80
	SetModeReq_MAV_MODE_STABILIZE_ARMED    uint8 = 208
	SetModeReq_MAV_MODE_MANUAL_DISARMED    uint8 = 64
	SetModeReq_MAV_MODE_MANUAL_ARMED       uint8 = 192
	SetModeReq_MAV_MODE_GUIDED_DISARMED    uint8 = 88
	SetModeReq_MAV_MODE_GUIDED_ARMED       uint8 = 216
	SetModeReq_MAV_MODE_AUTO_DISARMED      uint8 = 92
	SetModeReq_MAV_MODE_AUTO_ARMED         uint8 = 220
	SetModeReq_MAV_MODE_TEST_DISARMED      uint8 = 66
	SetModeReq_MAV_MODE_TEST_ARMED         uint8 = 194
)

type SetModeReq struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 MAV_MODE_PREFLIGHT=0,uint8 MAV_MODE_STABILIZE_DISARMED=80,uint8 MAV_MODE_STABILIZE_ARMED=208,uint8 MAV_MODE_MANUAL_DISARMED=64,uint8 MAV_MODE_MANUAL_ARMED=192,uint8 MAV_MODE_GUIDED_DISARMED=88,uint8 MAV_MODE_GUIDED_ARMED=216,uint8 MAV_MODE_AUTO_DISARMED=92,uint8 MAV_MODE_AUTO_ARMED=220,uint8 MAV_MODE_TEST_DISARMED=66,uint8 MAV_MODE_TEST_ARMED=194"`
	BaseMode        uint8
	CustomMode      string
}

type SetModeRes struct {
	msg.Package `ros:"mavros_msgs"`
	ModeSent    bool
}

type SetMode struct {
	msg.Package `ros:"mavros_msgs"`
	SetModeReq
	SetModeRes
}
