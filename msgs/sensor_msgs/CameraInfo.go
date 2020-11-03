package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type CameraInfo struct {
	msg.Package     `ros:"sensor_msgs"`
	Header          std_msgs.Header
	Height          uint32
	Width           uint32
	DistortionModel string
	D               []float64   `rosname:"D"`
	K               [9]float64  `rosname:"K"`
	R               [9]float64  `rosname:"R"`
	P               [12]float64 `rosname:"P"`
	BinningX        uint32
	BinningY        uint32
	Roi             RegionOfInterest
}
