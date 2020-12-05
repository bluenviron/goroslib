//nolint:golint
package visualization_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type InteractiveMarkerInit struct {
	msg.Package `ros:"visualization_msgs"`
	ServerId    string
	SeqNum      uint64
	Markers     []InteractiveMarker
}
