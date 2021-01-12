package ackermann_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

type AckermannDriveStamped struct { //nolint:golint
	msg.Package `ros:"ackermann_msgs"`
	Header      std_msgs.Header //nolint:golint
	Drive       AckermannDrive  //nolint:golint
}
