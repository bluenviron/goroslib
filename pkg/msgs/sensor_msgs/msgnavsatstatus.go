//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

const (
	NavSatStatus_STATUS_NO_FIX   int8   = -1
	NavSatStatus_STATUS_FIX      int8   = 0
	NavSatStatus_STATUS_SBAS_FIX int8   = 1
	NavSatStatus_STATUS_GBAS_FIX int8   = 2
	NavSatStatus_SERVICE_GPS     uint16 = 1
	NavSatStatus_SERVICE_GLONASS uint16 = 2
	NavSatStatus_SERVICE_COMPASS uint16 = 4
	NavSatStatus_SERVICE_GALILEO uint16 = 8
)

type NavSatStatus struct {
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"int8 STATUS_NO_FIX=-1,int8 STATUS_FIX=0,int8 STATUS_SBAS_FIX=1,int8 STATUS_GBAS_FIX=2,uint16 SERVICE_GPS=1,uint16 SERVICE_GLONASS=2,uint16 SERVICE_COMPASS=4,uint16 SERVICE_GALILEO=8"`
	Status          int8
	Service         uint16
}
