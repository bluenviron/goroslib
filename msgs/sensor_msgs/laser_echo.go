package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type LaserEcho struct {
	msg.Package `ros:"sensor_msgs"`
	Echoes      []float32
}
