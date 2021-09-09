//nolint:golint,lll
package trajectory_msgs

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
)

type JointTrajectoryPoint struct {
	msg.Package   `ros:"trajectory_msgs"`
	Positions     []float64
	Velocities    []float64
	Accelerations []float64
	Effort        []float64
	TimeFromStart time.Duration
}
