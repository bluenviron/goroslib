package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetCameraInfoReq struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	CameraInfo  CameraInfo //nolint:golint
}

type SetCameraInfoRes struct { //nolint:golint
	msg.Package   `ros:"sensor_msgs"`
	Success       bool   //nolint:golint
	StatusMessage string //nolint:golint
}
