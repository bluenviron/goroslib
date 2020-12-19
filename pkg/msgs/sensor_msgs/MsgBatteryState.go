package sensor_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	BatteryState_POWER_SUPPLY_STATUS_UNKNOWN               uint8 = 0 //nolint:golint
	BatteryState_POWER_SUPPLY_STATUS_CHARGING              uint8 = 1 //nolint:golint
	BatteryState_POWER_SUPPLY_STATUS_DISCHARGING           uint8 = 2 //nolint:golint
	BatteryState_POWER_SUPPLY_STATUS_NOT_CHARGING          uint8 = 3 //nolint:golint
	BatteryState_POWER_SUPPLY_STATUS_FULL                  uint8 = 4 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_UNKNOWN               uint8 = 0 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_GOOD                  uint8 = 1 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_OVERHEAT              uint8 = 2 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_DEAD                  uint8 = 3 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_OVERVOLTAGE           uint8 = 4 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_UNSPEC_FAILURE        uint8 = 5 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_COLD                  uint8 = 6 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE uint8 = 7 //nolint:golint
	BatteryState_POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE   uint8 = 8 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_UNKNOWN           uint8 = 0 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_NIMH              uint8 = 1 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_LION              uint8 = 2 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_LIPO              uint8 = 3 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_LIFE              uint8 = 4 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_NICD              uint8 = 5 //nolint:golint
	BatteryState_POWER_SUPPLY_TECHNOLOGY_LIMN              uint8 = 6 //nolint:golint
)

type BatteryState struct { //nolint:golint
	msg.Package           `ros:"sensor_msgs"`
	msg.Definitions       `ros:"uint8 POWER_SUPPLY_STATUS_UNKNOWN=0,uint8 POWER_SUPPLY_STATUS_CHARGING=1,uint8 POWER_SUPPLY_STATUS_DISCHARGING=2,uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3,uint8 POWER_SUPPLY_STATUS_FULL=4,uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0,uint8 POWER_SUPPLY_HEALTH_GOOD=1,uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2,uint8 POWER_SUPPLY_HEALTH_DEAD=3,uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4,uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5,uint8 POWER_SUPPLY_HEALTH_COLD=6,uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7,uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8,uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0,uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1,uint8 POWER_SUPPLY_TECHNOLOGY_LION=2,uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3,uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4,uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5,uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6"`
	Header                std_msgs.Header //nolint:golint
	Voltage               float32         //nolint:golint
	Temperature           float32         //nolint:golint
	Current               float32         //nolint:golint
	Charge                float32         //nolint:golint
	Capacity              float32         //nolint:golint
	DesignCapacity        float32         //nolint:golint
	Percentage            float32         //nolint:golint
	PowerSupplyStatus     uint8           //nolint:golint
	PowerSupplyHealth     uint8           //nolint:golint
	PowerSupplyTechnology uint8           //nolint:golint
	Present               bool            //nolint:golint
	CellVoltage           []float32       //nolint:golint
	CellTemperature       []float32       //nolint:golint
	Location              string          //nolint:golint
	SerialNumber          string          //nolint:golint
}
