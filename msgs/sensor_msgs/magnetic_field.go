package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type MagneticField struct {
	msgs.Package            `ros:"sensor_msgs"`
	Header                  std_msgs.Header
	MagneticField           geometry_msgs.Vector3
	MagneticFieldCovariance [9]float64
}
