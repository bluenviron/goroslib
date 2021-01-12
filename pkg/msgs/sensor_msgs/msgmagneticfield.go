package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type MagneticField struct { //nolint:golint
	msg.Package             `ros:"sensor_msgs"`
	Header                  std_msgs.Header       //nolint:golint
	MagneticField           geometry_msgs.Vector3 //nolint:golint
	MagneticFieldCovariance [9]float64            //nolint:golint
}
