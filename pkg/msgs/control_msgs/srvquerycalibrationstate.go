package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type QueryCalibrationStateReq struct { //nolint:golint
}

type QueryCalibrationStateRes struct { //nolint:golint
	IsCalibrated bool //nolint:golint
}

type QueryCalibrationState struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	QueryCalibrationStateReq
	QueryCalibrationStateRes
}
