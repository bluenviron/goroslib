package rosgraph_msgs

import (
	"github.com/aler9/goroslib/msg"
	"time"
)

type Clock struct {
	msg.Package `ros:"rosgraph_msgs"`
	Clock       time.Time
}
