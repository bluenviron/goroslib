package geometry_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type Inertia struct { //nolint:golint
	msg.Package `ros:"geometry_msgs"`
	M           float64 //nolint:golint
	Com         Vector3 //nolint:golint
	Ixx         float64 //nolint:golint
	Ixy         float64 //nolint:golint
	Ixz         float64 //nolint:golint
	Iyy         float64 //nolint:golint
	Iyz         float64 //nolint:golint
	Izz         float64 //nolint:golint
}
