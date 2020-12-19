package rosgraph_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Clock struct { //nolint:golint
	msg.Package `ros:"rosgraph_msgs"`
	Clock       time.Time //nolint:golint
}
