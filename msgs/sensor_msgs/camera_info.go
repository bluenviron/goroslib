package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type CameraInfo struct {
	msgs.Package    `ros:"sensor_msgs"`
	Header          std_msgs.Header
	Height          uint32
	Width           uint32
	DistortionModel string
	D               []float64
	K               [9]float64
	R               [9]float64
	P               [12]float64
	BinningX        uint32
	BinningY        uint32
	Roi             RegionOfInterest
}
