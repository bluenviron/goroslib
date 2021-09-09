//nolint:golint,lll
package sensor_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type CompressedImage struct {
	msg.Package `ros:"sensor_msgs"`
	Header      std_msgs.Header
	Format      string
	Data        []uint8
}
