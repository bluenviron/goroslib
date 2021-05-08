//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	Trajectory_MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS uint8 = 0
	Trajectory_MAV_TRAJECTORY_REPRESENTATION_BEZIER    uint8 = 1
)

type Trajectory struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS=0,uint8 MAV_TRAJECTORY_REPRESENTATION_BEZIER=1"`
	Header          std_msgs.Header
	Type            uint8
	Point1          PositionTarget `rosname:"point_1"`
	Point2          PositionTarget `rosname:"point_2"`
	Point3          PositionTarget `rosname:"point_3"`
	Point4          PositionTarget `rosname:"point_4"`
	Point5          PositionTarget `rosname:"point_5"`
	PointValid      [5]uint8
	Command         [5]uint16
	TimeHorizon     [5]float32
}
