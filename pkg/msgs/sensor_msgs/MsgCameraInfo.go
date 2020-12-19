package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type CameraInfo struct { //nolint:golint
	msg.Package     `ros:"sensor_msgs"`
	Header          std_msgs.Header  //nolint:golint
	Height          uint32           //nolint:golint
	Width           uint32           //nolint:golint
	DistortionModel string           //nolint:golint
	D               []float64        `rosname:"D"` //nolint:golint
	K               [9]float64       `rosname:"K"` //nolint:golint
	R               [9]float64       `rosname:"R"` //nolint:golint
	P               [12]float64      `rosname:"P"` //nolint:golint
	BinningX        uint32           //nolint:golint
	BinningY        uint32           //nolint:golint
	Roi             RegionOfInterest //nolint:golint
}
