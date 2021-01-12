package visualization_msgs //nolint:golint

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MarkerArray struct { //nolint:golint
	msg.Package `ros:"visualization_msgs"`
	Markers     []Marker //nolint:golint
}
