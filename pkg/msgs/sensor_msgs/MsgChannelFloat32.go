//nolint:golint
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ChannelFloat32 struct {
	msg.Package `ros:"sensor_msgs"`
	Name        string
	Values      []float32
}
