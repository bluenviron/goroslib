//nolint:golint
package visualization_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type MarkerArray struct {
	msg.Package `ros:"visualization_msgs"`
	Markers     []Marker
}
