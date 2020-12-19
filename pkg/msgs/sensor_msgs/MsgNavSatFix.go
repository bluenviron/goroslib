package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	NavSatFix_COVARIANCE_TYPE_UNKNOWN        uint8 = 0 //nolint:golint
	NavSatFix_COVARIANCE_TYPE_APPROXIMATED   uint8 = 1 //nolint:golint
	NavSatFix_COVARIANCE_TYPE_DIAGONAL_KNOWN uint8 = 2 //nolint:golint
	NavSatFix_COVARIANCE_TYPE_KNOWN          uint8 = 3 //nolint:golint
)

type NavSatFix struct { //nolint:golint
	msg.Package            `ros:"sensor_msgs"`
	msg.Definitions        `ros:"uint8 COVARIANCE_TYPE_UNKNOWN=0,uint8 COVARIANCE_TYPE_APPROXIMATED=1,uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2,uint8 COVARIANCE_TYPE_KNOWN=3"`
	Header                 std_msgs.Header //nolint:golint
	Status                 NavSatStatus    //nolint:golint
	Latitude               float64         //nolint:golint
	Longitude              float64         //nolint:golint
	Altitude               float64         //nolint:golint
	PositionCovariance     [9]float64      //nolint:golint
	PositionCovarianceType uint8           //nolint:golint
}
