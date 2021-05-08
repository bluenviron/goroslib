//nolint:golint
package geographic_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
	"github.com/aler9/goroslib/pkg/msgs/uuid_msgs"
)

type GeographicMapChanges struct {
	msg.Package `ros:"geographic_msgs"`
	Header      std_msgs.Header
	Diffs       GeographicMap
	Deletes     []uuid_msgs.UniqueID
}
