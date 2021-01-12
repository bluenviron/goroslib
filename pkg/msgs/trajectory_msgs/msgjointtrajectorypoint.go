package trajectory_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type JointTrajectoryPoint struct { //nolint:golint
	msg.Package   `ros:"trajectory_msgs"`
	Positions     []float64     //nolint:golint
	Velocities    []float64     //nolint:golint
	Accelerations []float64     //nolint:golint
	Effort        []float64     //nolint:golint
	TimeFromStart time.Duration //nolint:golint
}
