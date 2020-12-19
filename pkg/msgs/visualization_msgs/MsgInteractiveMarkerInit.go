package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type InteractiveMarkerInit struct { //nolint:golint
	msg.Package `ros:"visualization_msgs"`
	ServerId    string              //nolint:golint
	SeqNum      uint64              //nolint:golint
	Markers     []InteractiveMarker //nolint:golint
}
