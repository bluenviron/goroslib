//nolint:golint
package mavros_msgs

import (
	"time"

	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	ADSBVehicle_ALT_PRESSURE_QNH             uint8  = 0
	ADSBVehicle_ALT_GEOMETRIC                uint8  = 1
	ADSBVehicle_EMITTER_NO_INFO              uint8  = 0
	ADSBVehicle_EMITTER_LIGHT                uint8  = 1
	ADSBVehicle_EMITTER_SMALL                uint8  = 2
	ADSBVehicle_EMITTER_LARGE                uint8  = 3
	ADSBVehicle_EMITTER_HIGH_VORTEX_LARGE    uint8  = 4
	ADSBVehicle_EMITTER_HEAVY                uint8  = 5
	ADSBVehicle_EMITTER_HIGHLY_MANUV         uint8  = 6
	ADSBVehicle_EMITTER_ROTOCRAFT            uint8  = 7
	ADSBVehicle_EMITTER_UNASSIGNED           uint8  = 8
	ADSBVehicle_EMITTER_GLIDER               uint8  = 9
	ADSBVehicle_EMITTER_LIGHTER_AIR          uint8  = 10
	ADSBVehicle_EMITTER_PARACHUTE            uint8  = 11
	ADSBVehicle_EMITTER_ULTRA_LIGHT          uint8  = 12
	ADSBVehicle_EMITTER_UNASSIGNED2          uint8  = 13
	ADSBVehicle_EMITTER_UAV                  uint8  = 14
	ADSBVehicle_EMITTER_SPACE                uint8  = 15
	ADSBVehicle_EMITTER_UNASSGINED3          uint8  = 16
	ADSBVehicle_EMITTER_EMERGENCY_SURFACE    uint8  = 17
	ADSBVehicle_EMITTER_SERVICE_SURFACE      uint8  = 18
	ADSBVehicle_EMITTER_POINT_OBSTACLE       uint8  = 19
	ADSBVehicle_FLAG_VALID_COORDS            uint16 = 1
	ADSBVehicle_FLAG_VALID_ALTITUDE          uint16 = 2
	ADSBVehicle_FLAG_VALID_HEADING           uint16 = 4
	ADSBVehicle_FLAG_VALID_VELOCITY          uint16 = 8
	ADSBVehicle_FLAG_VALID_CALLSIGN          uint16 = 16
	ADSBVehicle_FLAG_VALID_SQUAWK            uint16 = 32
	ADSBVehicle_FLAG_SIMULATED               uint16 = 64
	ADSBVehicle_FLAG_VERTICAL_VELOCITY_VALID uint16 = 128
	ADSBVehicle_FLAG_BARO_VALID              uint16 = 256
	ADSBVehicle_FLAG_SOURCE_UAT              uint16 = 32768
)

type ADSBVehicle struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 ALT_PRESSURE_QNH=0,uint8 ALT_GEOMETRIC=1,uint8 EMITTER_NO_INFO=0,uint8 EMITTER_LIGHT=1,uint8 EMITTER_SMALL=2,uint8 EMITTER_LARGE=3,uint8 EMITTER_HIGH_VORTEX_LARGE=4,uint8 EMITTER_HEAVY=5,uint8 EMITTER_HIGHLY_MANUV=6,uint8 EMITTER_ROTOCRAFT=7,uint8 EMITTER_UNASSIGNED=8,uint8 EMITTER_GLIDER=9,uint8 EMITTER_LIGHTER_AIR=10,uint8 EMITTER_PARACHUTE=11,uint8 EMITTER_ULTRA_LIGHT=12,uint8 EMITTER_UNASSIGNED2=13,uint8 EMITTER_UAV=14,uint8 EMITTER_SPACE=15,uint8 EMITTER_UNASSGINED3=16,uint8 EMITTER_EMERGENCY_SURFACE=17,uint8 EMITTER_SERVICE_SURFACE=18,uint8 EMITTER_POINT_OBSTACLE=19,uint16 FLAG_VALID_COORDS=1,uint16 FLAG_VALID_ALTITUDE=2,uint16 FLAG_VALID_HEADING=4,uint16 FLAG_VALID_VELOCITY=8,uint16 FLAG_VALID_CALLSIGN=16,uint16 FLAG_VALID_SQUAWK=32,uint16 FLAG_SIMULATED=64,uint16 FLAG_VERTICAL_VELOCITY_VALID=128,uint16 FLAG_BARO_VALID=256,uint16 FLAG_SOURCE_UAT=32768"`
	Header          std_msgs.Header
	ICAOAddress     uint32 `rosname:"ICAO_address"`
	Callsign        string
	Latitude        float64
	Longitude       float64
	Altitude        float32
	Heading         float32
	HorVelocity     float32
	VerVelocity     float32
	AltitudeType    uint8
	EmitterType     uint8
	Tslc            time.Duration
	Flags           uint16
	Squawk          uint16
}
