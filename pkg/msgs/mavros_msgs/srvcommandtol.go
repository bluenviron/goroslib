//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandTOLReq struct {
	MinPitch  float32
	Yaw       float32
	Latitude  float32
	Longitude float32
	Altitude  float32
}

type CommandTOLRes struct {
	Success bool
	Result  uint8
}

type CommandTOL struct {
	msg.Package `ros:"mavros_msgs"`
	CommandTOLReq
	CommandTOLRes
}
