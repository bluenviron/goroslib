//nolint:golint,lll
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetCameraInfoReq struct {
	CameraInfo CameraInfo
}

type SetCameraInfoRes struct {
	Success       bool
	StatusMessage string
}

type SetCameraInfo struct {
	msg.Package `ros:"sensor_msgs"`
	SetCameraInfoReq
	SetCameraInfoRes
}
