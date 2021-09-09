//nolint:golint,lll
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type QueryCalibrationStateReq struct{}

type QueryCalibrationStateRes struct {
	IsCalibrated bool
}

type QueryCalibrationState struct {
	msg.Package `ros:"control_msgs"`
	QueryCalibrationStateReq
	QueryCalibrationStateRes
}
