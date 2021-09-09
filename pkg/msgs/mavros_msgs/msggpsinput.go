//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	GPSINPUT_GPS_FIX_TYPE_NO_GPS     uint8 = 0
	GPSINPUT_GPS_FIX_TYPE_NO_FIX     uint8 = 1
	GPSINPUT_GPS_FIX_TYPE_2D_FIX     uint8 = 2
	GPSINPUT_GPS_FIX_TYPE_3D_FIX     uint8 = 3
	GPSINPUT_GPS_FIX_TYPE_DGPS       uint8 = 4
	GPSINPUT_GPS_FIX_TYPE_RTK_FLOATR uint8 = 5
	GPSINPUT_GPS_FIX_TYPE_RTK_FIXEDR uint8 = 6
	GPSINPUT_GPS_FIX_TYPE_STATIC     uint8 = 7
	GPSINPUT_GPS_FIX_TYPE_PPP        uint8 = 8
)

type GPSINPUT struct {
	msg.Package       `ros:"mavros_msgs"`
	msg.Definitions   `ros:"uint8 GPS_FIX_TYPE_NO_GPS=0,uint8 GPS_FIX_TYPE_NO_FIX=1,uint8 GPS_FIX_TYPE_2D_FIX=2,uint8 GPS_FIX_TYPE_3D_FIX=3,uint8 GPS_FIX_TYPE_DGPS=4,uint8 GPS_FIX_TYPE_RTK_FLOATR=5,uint8 GPS_FIX_TYPE_RTK_FIXEDR=6,uint8 GPS_FIX_TYPE_STATIC=7,uint8 GPS_FIX_TYPE_PPP=8"`
	Header            std_msgs.Header
	FixType           uint8
	GpsId             uint8
	IgnoreFlags       uint16
	TimeWeekMs        uint32
	TimeWeek          uint16
	Lat               int32
	Lon               int32
	Alt               float32
	Hdop              float32
	Vdop              float32
	Vn                float32
	Ve                float32
	Vd                float32
	SpeedAccuracy     float32
	HorizAccuracy     float32
	VertAccuracy      float32
	SatellitesVisible uint8
}
