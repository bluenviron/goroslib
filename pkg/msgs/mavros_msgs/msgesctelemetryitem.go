//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type ESCTelemetryItem struct {
	msg.Package  `ros:"mavros_msgs"`
	Header       std_msgs.Header
	Temperature  float32
	Voltage      float32
	Current      float32
	Totalcurrent float32
	Rpm          int32
	Count        uint16
}