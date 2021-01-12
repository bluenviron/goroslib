package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type LaserScan struct { //nolint:golint
	msg.Package    `ros:"sensor_msgs"`
	Header         std_msgs.Header //nolint:golint
	AngleMin       float32         //nolint:golint
	AngleMax       float32         //nolint:golint
	AngleIncrement float32         //nolint:golint
	TimeIncrement  float32         //nolint:golint
	ScanTime       float32         //nolint:golint
	RangeMin       float32         //nolint:golint
	RangeMax       float32         //nolint:golint
	Ranges         []float32       //nolint:golint
	Intensities    []float32       //nolint:golint
}
