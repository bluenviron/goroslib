//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LaserEcho struct {
	msg.Package `ros:"sensor_msgs"`
	Echoes      []float32
}
