//nolint:golint,lll
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
)

type ParamValue struct {
	msg.Package `ros:"mavros_msgs"`
	Integer     int64
	Real        float64
}
