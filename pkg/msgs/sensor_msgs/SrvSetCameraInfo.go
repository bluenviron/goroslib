package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type SetCameraInfoReq struct { //nolint:golint
	CameraInfo CameraInfo //nolint:golint
}

type SetCameraInfoRes struct { //nolint:golint
	Success       bool   //nolint:golint
	StatusMessage string //nolint:golint
}

type SetCameraInfo struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	SetCameraInfoReq
	SetCameraInfoRes
}
