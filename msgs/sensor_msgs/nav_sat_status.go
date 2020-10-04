package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
)

type NavSatStatus struct {
	msg.Package     `ros:"sensor_msgs"`
	msg.Definitions `ros:"int8 STATUS_NO_FIX=-1,int8 STATUS_FIX=0,int8 STATUS_SBAS_FIX=1,int8 STATUS_GBAS_FIX=2,uint16 SERVICE_GPS=1,uint16 SERVICE_GLONASS=2,uint16 SERVICE_COMPASS=4,uint16 SERVICE_GALILEO=8"`
	Status          int8
	Service         uint16
}
