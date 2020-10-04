package sensor_msgs

import (
	"github.com/aler9/goroslib/msg"
	"github.com/aler9/goroslib/msgs/std_msgs"
)

type BatteryState struct {
	msg.Package           `ros:"sensor_msgs"`
	msg.Definitions       `ros:"uint8 POWER_SUPPLY_STATUS_UNKNOWN=0,uint8 POWER_SUPPLY_STATUS_CHARGING=1,uint8 POWER_SUPPLY_STATUS_DISCHARGING=2,uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3,uint8 POWER_SUPPLY_STATUS_FULL=4,uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0,uint8 POWER_SUPPLY_HEALTH_GOOD=1,uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2,uint8 POWER_SUPPLY_HEALTH_DEAD=3,uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4,uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5,uint8 POWER_SUPPLY_HEALTH_COLD=6,uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7,uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8,uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0,uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1,uint8 POWER_SUPPLY_TECHNOLOGY_LION=2,uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3,uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4,uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5,uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6"`
	Header                std_msgs.Header
	Voltage               float32
	Temperature           float32
	Current               float32
	Charge                float32
	Capacity              float32
	DesignCapacity        float32
	Percentage            float32
	PowerSupplyStatus     uint8
	PowerSupplyHealth     uint8
	PowerSupplyTechnology uint8
	Present               bool
	CellVoltage           []float32
	CellTemperature       []float32
	Location              string
	SerialNumber          string
}
