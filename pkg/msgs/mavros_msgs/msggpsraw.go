//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	GPSRAW_GPS_FIX_TYPE_NO_GPS     uint8 = 0
	GPSRAW_GPS_FIX_TYPE_NO_FIX     uint8 = 1
	GPSRAW_GPS_FIX_TYPE_2D_FIX     uint8 = 2
	GPSRAW_GPS_FIX_TYPE_3D_FIX     uint8 = 3
	GPSRAW_GPS_FIX_TYPE_DGPS       uint8 = 4
	GPSRAW_GPS_FIX_TYPE_RTK_FLOATR uint8 = 5
	GPSRAW_GPS_FIX_TYPE_RTK_FIXEDR uint8 = 6
	GPSRAW_GPS_FIX_TYPE_STATIC     uint8 = 7
	GPSRAW_GPS_FIX_TYPE_PPP        uint8 = 8
)

type GPSRAW struct {
	msg.Package       `ros:"mavros_msgs"`
	msg.Definitions   `ros:"uint8 GPS_FIX_TYPE_NO_GPS=0,uint8 GPS_FIX_TYPE_NO_FIX=1,uint8 GPS_FIX_TYPE_2D_FIX=2,uint8 GPS_FIX_TYPE_3D_FIX=3,uint8 GPS_FIX_TYPE_DGPS=4,uint8 GPS_FIX_TYPE_RTK_FLOATR=5,uint8 GPS_FIX_TYPE_RTK_FIXEDR=6,uint8 GPS_FIX_TYPE_STATIC=7,uint8 GPS_FIX_TYPE_PPP=8"`
	Header            std_msgs.Header
	FixType           uint8
	Lat               int32
	Lon               int32
	Alt               int32
	Eph               uint16
	Epv               uint16
	Vel               uint16
	Cog               uint16
	SatellitesVisible uint8
	AltEllipsoid      int32
	HAcc              uint32
	VAcc              uint32
	VelAcc            uint32
	HdgAcc            int32
	DgpsNumch         uint8
	DgpsAge           uint32
}
