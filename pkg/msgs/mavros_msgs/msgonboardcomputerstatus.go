//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type OnboardComputerStatus struct {
	msg.Package      `ros:"mavros_msgs"`
	Header           std_msgs.Header
	Component        uint8
	Uptime           uint32
	Type             uint8
	CpuCores         [8]uint8
	CpuCombined      [10]uint8
	GpuCores         [4]uint8
	GpuCombined      [10]uint8
	TemperatureBoard int8
	TemperatureCore  [8]int8
	FanSpeed         [4]int16
	RamUsage         uint32
	RamTotal         uint32
	StorageType      [4]uint32
	StorageUsage     [4]uint32
	StorageTotal     [4]uint32
	LinkType         [6]uint32
	LinkTxRate       [6]uint32
	LinkRxRate       [6]uint32
	LinkTxMax        [6]uint32
	LinkRxMax        [6]uint32
}
