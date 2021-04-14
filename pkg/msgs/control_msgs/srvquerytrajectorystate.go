//nolint:golint
package control_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type QueryTrajectoryStateReq struct {
	Time time.Time
}

type QueryTrajectoryStateRes struct {
	Name         []string
	Position     []float64
	Velocity     []float64
	Acceleration []float64
}

type QueryTrajectoryState struct {
	msg.Package `ros:"control_msgs"`
	QueryTrajectoryStateReq
	QueryTrajectoryStateRes
}
