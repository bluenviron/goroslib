package control_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type QueryTrajectoryStateReq struct { //nolint:golint
	Time time.Time //nolint:golint
}

type QueryTrajectoryStateRes struct { //nolint:golint
	Name         []string  //nolint:golint
	Position     []float64 //nolint:golint
	Velocity     []float64 //nolint:golint
	Acceleration []float64 //nolint:golint
}

type QueryTrajectoryState struct { //nolint:golint
	msg.Package `ros:"control_msgs"`
	QueryTrajectoryStateReq
	QueryTrajectoryStateRes
}
