//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetCameraInfoReq struct {
	msg.Package `ros:"sensor_msgs"`
	CameraInfo  CameraInfo
}

type SetCameraInfoRes struct {
	msg.Package   `ros:"sensor_msgs"`
	Success       bool
	StatusMessage string
}
