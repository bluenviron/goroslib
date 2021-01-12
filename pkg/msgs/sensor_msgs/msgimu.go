package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/geometry_msgs"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type Imu struct { //nolint:golint
	msg.Package                  `ros:"sensor_msgs"`
	Header                       std_msgs.Header          //nolint:golint
	Orientation                  geometry_msgs.Quaternion //nolint:golint
	OrientationCovariance        [9]float64               //nolint:golint
	AngularVelocity              geometry_msgs.Vector3    //nolint:golint
	AngularVelocityCovariance    [9]float64               //nolint:golint
	LinearAcceleration           geometry_msgs.Vector3    //nolint:golint
	LinearAccelerationCovariance [9]float64               //nolint:golint
}
