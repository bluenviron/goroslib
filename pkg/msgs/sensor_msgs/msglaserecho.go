package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type LaserEcho struct { //nolint:golint
	msg.Package `ros:"sensor_msgs"`
	Echoes      []float32 //nolint:golint
}
