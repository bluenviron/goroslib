//nolint:golint
package rosgraph_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"time"
)

type Clock struct {
	msg.Package `ros:"rosgraph_msgs"`
	Clock       time.Time
}
