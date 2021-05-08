//nolint:golint
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/msgs/uuid_msgs"
)

type RoutePath struct {
	msg.Package `ros:"geographic_msgs"`
	Header      std_msgs.Header
	Network     uuid_msgs.UniqueID
	Segments    []uuid_msgs.UniqueID
	Props       []KeyValue
}
