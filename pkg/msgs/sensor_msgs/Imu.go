package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Imu struct {
	msg.Package                  `ros:"sensor_msgs"`
	Header                       std_msgs.Header
	Orientation                  geometry_msgs.Quaternion
	OrientationCovariance        [9]float64
	AngularVelocity              geometry_msgs.Vector3
	AngularVelocityCovariance    [9]float64
	LinearAcceleration           geometry_msgs.Vector3
	LinearAccelerationCovariance [9]float64
}
