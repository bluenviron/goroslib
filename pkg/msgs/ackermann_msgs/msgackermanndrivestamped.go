//nolint:golint,lll
package ackermann_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type AckermannDriveStamped struct {
	msg.Package `ros:"ackermann_msgs"`
	Header      std_msgs.Header
	Drive       AckermannDrive
}
