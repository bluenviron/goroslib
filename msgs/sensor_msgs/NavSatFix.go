package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

const (
	NavSatFix_COVARIANCE_TYPE_UNKNOWN        uint8 = 0
	NavSatFix_COVARIANCE_TYPE_APPROXIMATED   uint8 = 1
	NavSatFix_COVARIANCE_TYPE_DIAGONAL_KNOWN uint8 = 2
	NavSatFix_COVARIANCE_TYPE_KNOWN          uint8 = 3
)

type NavSatFix struct {
	msg.Package            `ros:"sensor_msgs"`
	msg.Definitions        `ros:"uint8 COVARIANCE_TYPE_UNKNOWN=0,uint8 COVARIANCE_TYPE_APPROXIMATED=1,uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2,uint8 COVARIANCE_TYPE_KNOWN=3"`
	Header                 std_msgs.Header
	Status                 NavSatStatus
	Latitude               float64
	Longitude              float64
	Altitude               float64
	PositionCovariance     [9]float64
	PositionCovarianceType uint8
}
