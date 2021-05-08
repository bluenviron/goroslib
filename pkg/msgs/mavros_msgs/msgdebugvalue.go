//nolint:golint
package mavros_msgs

import (
	"github.com/aler9/goroslib/pkg/msg"
	"github.com/aler9/goroslib/pkg/msgs/std_msgs"
)

const (
	DebugValue_TYPE_DEBUG             uint8 = 0
	DebugValue_TYPE_DEBUG_VECT        uint8 = 1
	DebugValue_TYPE_DEBUG_ARRAY       uint8 = 2
	DebugValue_TYPE_NAMED_VALUE_FLOAT uint8 = 3
	DebugValue_TYPE_NAMED_VALUE_INT   uint8 = 4
)

type DebugValue struct {
	msg.Package     `ros:"mavros_msgs"`
	msg.Definitions `ros:"uint8 TYPE_DEBUG=0,uint8 TYPE_DEBUG_VECT=1,uint8 TYPE_DEBUG_ARRAY=2,uint8 TYPE_NAMED_VALUE_FLOAT=3,uint8 TYPE_NAMED_VALUE_INT=4"`
	Header          std_msgs.Header
	Index           int32
	Name            string
	ValueFloat      float32
	ValueInt        int32
	Data            []float32
	Type            uint8
}
