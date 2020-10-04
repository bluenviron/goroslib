package sensor_msgs

import (
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type MultiEchoLaserScan struct {
	msgs.Package   `ros:"sensor_msgs"`
	Header         std_msgs.Header
	AngleMin       float32
	AngleMax       float32
	AngleIncrement float32
	TimeIncrement  float32
	ScanTime       float32
	RangeMin       float32
	RangeMax       float32
	Ranges         []LaserEcho
	Intensities    []LaserEcho
}
