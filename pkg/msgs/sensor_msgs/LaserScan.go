//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type LaserScan struct {
	msg.Package    `ros:"sensor_msgs"`
	Header         std_msgs.Header
	AngleMin       float32
	AngleMax       float32
	AngleIncrement float32
	TimeIncrement  float32
	ScanTime       float32
	RangeMin       float32
	RangeMax       float32
	Ranges         []float32
	Intensities    []float32
}
