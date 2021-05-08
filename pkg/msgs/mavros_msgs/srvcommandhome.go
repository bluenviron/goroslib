//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type CommandHomeReq struct {
	CurrentGps bool
	Yaw        float32
	Latitude   float32
	Longitude  float32
	Altitude   float32
}

type CommandHomeRes struct {
	Success bool
	Result  uint8
}

type CommandHome struct {
	msg.Package `ros:"mavros_msgs"`
	CommandHomeReq
	CommandHomeRes
}
