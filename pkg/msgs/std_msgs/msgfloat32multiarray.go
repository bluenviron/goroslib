//autogenerated:yes
//nolint:revive,lll
package std_msgs

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
)

type Float32MultiArray struct {
	msg.Package `ros:"std_msgs"`
	Layout      MultiArrayLayout
	Data        []float32
}
